"""
cbf_node.py

ROS2 node that monitors end-effector position and enforces
CBF safety constraints for obstacle avoidance.

Obstacles:
    1. TV    - static sphere
    2. Human - static sphere (dynamic tracking: future work)

Architecture:
    /joint_states -> Forward kinematics -> p_ee -> CBF check -> log h values
    /arm_controller/follow_joint_trajectory (action server)

Topics:
    Subscribes:
        /joint_states (sensor_msgs/JointState)
    Publishes:
        /cbf/h_tv          (std_msgs/Float64) barrier value for TV
        /cbf/h_human       (std_msgs/Float64) barrier value for human
        /cbf/position_safe (std_msgs/Bool)    True if p_ee is outside all spheres (position only)
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, CancelResponse, GoalResponse
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Bool
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np

from my_robot_controller.kdl_kinematics import KDLKinematics
from my_robot_controller.barrier_function import BarrierFunction
from my_robot_controller.cbf_filter import CBFFilter


class CBFNode(Node):

    JOINT_NAMES = [
        'shoulder_roll',
        'shoulder_pitch',
        'elbow_roll',
        'elbow_pitch',
        'wrist_roll',
        'wrist_pitch',
    ]

    def __init__(self):
        super().__init__('cbf_node')

        # Load parameters
        self._declare_parameters()

        control_rate  = self.get_parameter('control_rate').value

        tv_center     = self.get_parameter('tv.p_center').value
        tv_r          = self.get_parameter('tv.r_safe').value
        tv_gamma      = self.get_parameter('tv.gamma').value

        human_center  = self.get_parameter('human.p_center').value
        human_r       = self.get_parameter('human.r_safe').value
        human_gamma   = self.get_parameter('human.gamma').value

        # KDL kinematics
        self.declare_parameter('robot_description', '')
        robot_description = self.get_parameter('robot_description').value
        if not robot_description:
            self.get_logger().fatal(
                'robot_description parameter is empty. '
                'Make sure robot_state_publisher is running.'
            )
            raise RuntimeError('robot_description not found')

        self._kin = KDLKinematics(robot_description)
        self.get_logger().info('KDL kinematics initialized.')

        # CBF obstacles
        self._tv_barrier = BarrierFunction(
            name='tv',
            p_obstacle=list(tv_center),
            r_safe=tv_r,
            gamma=tv_gamma,
        )
        self._human_barrier = BarrierFunction(
            name='human',
            p_obstacle=list(human_center),
            r_safe=human_r,
            gamma=human_gamma,
        )
        self._cbf_filter = CBFFilter([self._tv_barrier, self._human_barrier])
        self.get_logger().info(
            f'CBF obstacles: '
            f'TV @ {tv_center} r={tv_r}m | '
            f'Human @ {human_center} r={human_r}m'
        )

        # Joint state storage
        self._joint_positions  = np.zeros(6)
        self._joint_velocities = np.zeros(6)
        self._last_js_time     = self.get_clock().now()
        self._dt               = 1.0 / control_rate

        # Subscribers
        self._js_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self._joint_state_callback,
            10,
        )

        # Publishers
        # NOTE: /cbf/position_safe reflects h >= 0 (geometry only, no velocity).
        #       The CBF filter itself uses cbf_condition() which is velocity-aware
        #       and is the authoritative safety check during trajectory execution.
        self._pub_h_tv    = self.create_publisher(Float64, '/cbf/h_tv',          10)
        self._pub_h_human = self.create_publisher(Float64, '/cbf/h_human',       10)
        self._pub_safe    = self.create_publisher(Bool,    '/cbf/position_safe',  10)

        # Action server — intercepts MoveIt goals
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory',
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
        )

        # Action client — forwards safe goals to real controller
        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/arm_controller_safe/follow_joint_trajectory',
        )

        # Monitoring timer
        self._monitor_timer = self.create_timer(self._dt, self._monitor_callback)

        self.get_logger().info('CBF node started.')

    # Parameter declaration

    def _declare_parameters(self):
        self.declare_parameter('control_rate', 10.0)

        self.declare_parameter('tv.p_center', [0.25, 0.00, 0.32])
        self.declare_parameter('tv.r_safe',   0.15)
        self.declare_parameter('tv.gamma',    1.0)

        self.declare_parameter('human.p_center', [0.0, 0.5, 0.5])
        self.declare_parameter('human.r_safe',   0.3)
        self.declare_parameter('human.gamma',    1.5)

    # Joint state callback

    def _joint_state_callback(self, msg: JointState):
        """Extract arm joint positions in JOINT_NAMES order.
        Velocity is estimated numerically using actual time between messages.
        """
        now = self.get_clock().now()
        dt_actual = (now - self._last_js_time).nanoseconds * 1e-9

        for i, name in enumerate(self.JOINT_NAMES):
            if name in msg.name:
                idx   = msg.name.index(name)
                q_new = msg.position[idx]

                # Use actual elapsed time, not fixed _dt, for velocity estimate
                if dt_actual > 0.001:
                    self._joint_velocities[i] = (
                        (q_new - self._joint_positions[i]) / dt_actual
                    )
                self._joint_positions[i] = q_new

        self._last_js_time = now

    # Monitor callback

    def _monitor_callback(self):
        """
        Runs at control_rate Hz.
        Computes FK, evaluates h values, publishes monitoring topics.
        NOTE: position_safe is a geometric check only (h >= 0).
              It does not account for velocity direction.
              Authoritative safety enforcement happens in _execute_callback.
        """
        q = self._joint_positions.copy()

        try:
            p_ee = self._kin.fk(q)
        except Exception as e:
            self.get_logger().warn(f'FK failed: {e}', throttle_duration_sec=5.0)
            return

        h_tv    = self._tv_barrier.h(p_ee)
        h_human = self._human_barrier.h(p_ee)
        position_safe = (h_tv >= 0.0) and (h_human >= 0.0)

        msg_tv    = Float64(); msg_tv.data    = h_tv
        msg_human = Float64(); msg_human.data = h_human
        msg_safe  = Bool();    msg_safe.data  = position_safe

        self._pub_h_tv.publish(msg_tv)
        self._pub_h_human.publish(msg_human)
        self._pub_safe.publish(msg_safe)

        if h_tv < 0.0:
            self.get_logger().warn(
                f'CBF VIOLATION — TV: h={h_tv:.4f} p_ee={p_ee}',
                throttle_duration_sec=1.0,
            )
        if h_human < 0.0:
            self.get_logger().warn(
                f'CBF VIOLATION — Human: h={h_human:.4f} p_ee={p_ee}',
                throttle_duration_sec=1.0,
            )

    # Action server callbacks

    def _goal_callback(self, goal_request):
        self.get_logger().info('CBF node received trajectory goal.')
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle):
        self.get_logger().info('CBF node received cancel request.')
        return CancelResponse.ACCEPT

    async def _execute_callback(self, goal_handle):
        """
        Filter each waypoint through CBF before forwarding.
        Aborts the goal if FK/Jacobian fails on any waypoint — forwarding
        an unfiltered waypoint would bypass the safety guarantee.
        """
        trajectory = goal_handle.request.trajectory
        self.get_logger().info(
            f'Filtering trajectory with {len(trajectory.points)} waypoints.'
        )

        filtered_points = []
        cbf_activated   = False

        for point in trajectory.points:
            # Build joint position array in JOINT_NAMES order
            u_nom = np.zeros(6)
            for i, name in enumerate(self.JOINT_NAMES):
                if name in trajectory.joint_names:
                    idx       = trajectory.joint_names.index(name)
                    u_nom[i]  = point.positions[idx]

            # Forward kinematics at commanded position
            # Abort on failure — passing an unfiltered point is unsafe
            try:
                p_ee = self._kin.fk(u_nom)
                Jv   = self._kin.jacobian_linear(u_nom)
            except Exception as e:
                self.get_logger().error(
                    f'FK/Jacobian failed on waypoint, aborting goal: {e}'
                )
                goal_handle.abort()
                return FollowJointTrajectory.Result()

            # Use current joint velocities as approximation for this waypoint.
            # NOTE: this is the velocity at goal-receipt time, not per-waypoint.
            # For slow trajectories this is acceptable; for fast ones, consider
            # propagating velocities from the trajectory points themselves.
            q_dot = self._joint_velocities.copy()

            # Apply CBF filter
            u_safe, h_values, modified = self._cbf_filter.filter(
                u_nom, p_ee, q_dot, Jv
            )

            # modified=False can mean "already safe" OR "QP failed, returned u_nom"
            # The filter logs a warning internally on QP failure, so we only
            # log the CBF-activated case here.
            if modified:
                cbf_activated = True
                self.get_logger().warn(
                    f'CBF modified waypoint: '
                    f'h_tv={h_values[0]:.4f} h_human={h_values[1]:.4f}'
                )

            safe_point = JointTrajectoryPoint()
            safe_point.time_from_start = point.time_from_start
            safe_point.positions       = list(u_safe)
            if point.velocities:
                safe_point.velocities  = point.velocities
            filtered_points.append(safe_point)

        if cbf_activated:
            self.get_logger().warn('CBF filter modified trajectory for safety.')

        # Forward filtered trajectory to real controller
        safe_trajectory            = JointTrajectory()
        safe_trajectory.joint_names = trajectory.joint_names
        safe_trajectory.points      = filtered_points

        new_goal          = FollowJointTrajectory.Goal()
        new_goal.trajectory = safe_trajectory

        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(
                'arm_controller_safe action server not available.'
            )
            goal_handle.abort()
            return FollowJointTrajectory.Result()

        send_goal_future = await self._action_client.send_goal_async(new_goal)

        if not send_goal_future.accepted:
            self.get_logger().error('Safe trajectory goal rejected.')
            goal_handle.abort()
            return FollowJointTrajectory.Result()

        result_future = await send_goal_future.get_result_async()

        goal_handle.succeed()
        return FollowJointTrajectory.Result()


# Main

def main(args=None):
    rclpy.init(args=args)
    node = CBFNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()