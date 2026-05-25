"""
cbf_node.py

ROS2 node that monitors end-effector position and enforces
CBF safety constraints for obstacle avoidance.

Obstacles:
    1. TV    — static sphere
    2. Human — static sphere (dynamic tracking: future work)

Architecture:
    /joint_states → FK → p_ee → CBF check → log h values
    /arm_controller/follow_joint_trajectory (action server)
        → CBF filter on each waypoint
        → forward safe goal to /arm_controller_safe/follow_joint_trajectory

Topics:
    Subscribes:
        /joint_states (sensor_msgs/JointState)
    Publishes:
        /cbf/h_tv    (std_msgs/Float64) — barrier value for TV
        /cbf/h_human (std_msgs/Float64) — barrier value for human
        /cbf/safe    (std_msgs/Bool)    — True if all constraints satisfied
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, CancelResponse, GoalResponse
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Bool
from control_msgs.action import FollowJointTrajectory
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

        joint_names   = self.get_parameter('joint_names').value
        control_rate  = self.get_parameter('control_rate').value

        tv_center     = self.get_parameter('tv.p_center').value
        tv_r          = self.get_parameter('tv.r_safe').value
        tv_gamma      = self.get_parameter('tv.gamma').value

        human_center  = self.get_parameter('human.p_center').value
        human_r       = self.get_parameter('human.r_safe').value
        human_gamma   = self.get_parameter('human.gamma').value

        # KDL kinematics
        # Load robot_description from parameter server
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
        self._joint_positions = np.zeros(6)
        self._joint_velocities = np.zeros(6)
        self._joint_names_received = []
        self._q_prev = np.zeros(6)   # for numerical velocity estimation
        self._dt = 1.0 / control_rate

        # Subscribers
        self._js_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self._joint_state_callback,
            10,
        )

        # Publishers 
        self._pub_h_tv = self.create_publisher(
            Float64, '/cbf/h_tv', 10)
        self._pub_h_human = self.create_publisher(
            Float64, '/cbf/h_human', 10)
        self._pub_safe = self.create_publisher(
            Bool, '/cbf/safe', 10)

        # Action server (intercepts MoveIt goals) 
        # MoveIt sends to /arm_controller/follow_joint_trajectory
        # This node filters waypoints and forwards to the real controller
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory',
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
        )

        # ── Action client (forwards safe goals to real controller) ─────────
        # joint_trajectory_controller must be remapped to _safe namespace
        # in controllers.yaml or launch file
        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/arm_controller_safe/follow_joint_trajectory',
        )

        # ── Monitoring timer ──────────────────────────────────────────────
        self._monitor_timer = self.create_timer(
            self._dt, self._monitor_callback)

        self.get_logger().info('CBF node started.')

    # ── Parameter declaration ─────────────────────────────────────────────

    def _declare_parameters(self):
        self.declare_parameter('joint_names', self.JOINT_NAMES)
        self.declare_parameter('control_rate', 10.0)

        self.declare_parameter('tv.p_center', [0.4, 0.3, 0.3])
        self.declare_parameter('tv.r_safe', 0.15)
        self.declare_parameter('tv.gamma', 1.0)

        self.declare_parameter('human.p_center', [0.0, 0.5, 0.5])
        self.declare_parameter('human.r_safe', 0.3)
        self.declare_parameter('human.gamma', 1.5)

    # ── Joint state callback ──────────────────────────────────────────────

    def _joint_state_callback(self, msg: JointState):
        """Extract arm joint positions in JOINT_NAMES order."""
        self._joint_names_received = list(msg.name)
        for i, name in enumerate(self.JOINT_NAMES):
            if name in msg.name:
                idx = msg.name.index(name)
                q_new = msg.position[idx]

                # Numerical velocity estimation
                self._joint_velocities[i] = (
                    (q_new - self._joint_positions[i]) / self._dt
                )
                self._joint_positions[i] = q_new

    # Monitor callback

    def _monitor_callback(self):
        """
        Runs at control_rate Hz.
        Computes FK, evaluates CBF conditions, publishes h values.
        """
        q = self._joint_positions.copy()

        # Forward kinematics
        try:
            p_ee = self._kin.fk(q)
        except Exception as e:
            self.get_logger().warn(f'FK failed: {e}', throttle_duration_sec=5.0)
            return

        # Evaluate barrier values
        h_tv    = self._tv_barrier.h(p_ee)
        h_human = self._human_barrier.h(p_ee)
        is_safe = (h_tv >= 0.0) and (h_human >= 0.0)

        # Publish
        msg_tv = Float64()
        msg_tv.data = h_tv
        self._pub_h_tv.publish(msg_tv)

        msg_human = Float64()
        msg_human.data = h_human
        self._pub_h_human.publish(msg_human)

        msg_safe = Bool()
        msg_safe.data = is_safe
        self._pub_safe.publish(msg_safe)

        # Warn if unsafe
        if h_tv < 0.0:
            self.get_logger().warn(
                f'CBF VIOLATION — TV: h={h_tv:.4f} '
                f'p_ee={p_ee}',
                throttle_duration_sec=1.0
            )
        if h_human < 0.0:
            self.get_logger().warn(
                f'CBF VIOLATION — Human: h={h_human:.4f} '
                f'p_ee={p_ee}',
                throttle_duration_sec=1.0
            )

    # ── Action server callbacks ───────────────────────────────────────────

    def _goal_callback(self, goal_request):
        self.get_logger().info('CBF node received trajectory goal.')
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle):
        self.get_logger().info('CBF node received cancel request.')
        return CancelResponse.ACCEPT

    async def _execute_callback(self, goal_handle):
        """
        Filter each waypoint through CBF before forwarding.
        """
        trajectory = goal_handle.request.trajectory
        self.get_logger().info(
            f'Filtering trajectory with '
            f'{len(trajectory.points)} waypoints.'
        )

        q_dot = self._joint_velocities.copy()
        filtered_points = []
        cbf_activated = False

        for point in trajectory.points:
            # Build joint position array in JOINT_NAMES order
            u_nom = np.zeros(6)
            for i, name in enumerate(self.JOINT_NAMES):
                if name in trajectory.joint_names:
                    idx = trajectory.joint_names.index(name)
                    u_nom[i] = point.positions[idx]

            # Forward kinematics at commanded position
            try:
                p_ee = self._kin.fk(u_nom)
                Jv   = self._kin.jacobian_linear(u_nom)
            except Exception as e:
                self.get_logger().warn(f'FK/Jacobian failed: {e}')
                filtered_points.append(point)
                continue

            # Apply CBF filter
            u_safe, h_values, modified = self._cbf_filter.filter(
                u_nom, p_ee, q_dot, Jv
            )

            if modified:
                cbf_activated = True
                self.get_logger().warn(
                    f'CBF modified waypoint: '
                    f'h_tv={h_values[0]:.4f} '
                    f'h_human={h_values[1]:.4f}'
                )

            # Rebuild point with safe positions
            from trajectory_msgs.msg import JointTrajectoryPoint
            safe_point = JointTrajectoryPoint()
            safe_point.time_from_start = point.time_from_start
            safe_point.positions = list(u_safe)
            if point.velocities:
                safe_point.velocities = point.velocities
            filtered_points.append(safe_point)

        if cbf_activated:
            self.get_logger().warn(
                'CBF filter modified trajectory for safety.'
            )

        # Forward filtered trajectory to real controller
        from control_msgs.action import FollowJointTrajectory as FJT
        from trajectory_msgs.msg import JointTrajectory

        safe_trajectory = JointTrajectory()
        safe_trajectory.joint_names = trajectory.joint_names
        safe_trajectory.points = filtered_points

        new_goal = FJT.Goal()
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

        # Wait for result
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
