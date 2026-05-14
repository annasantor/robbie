from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
import os
import yaml


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    with open(absolute_file_path) as f:
        return yaml.safe_load(f)


def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    sim_time = {'use_sim_time': True}

    # ── Resolved paths ────────────────────────────────────────────────────────
    initial_positions_file = os.path.join(
        get_package_share_directory('my_robot_moveit_config'),
        'config', 'initial_positions.yaml'
    )


    pkg_desc = get_package_share_directory('my_robot_description')

    controller_manager_file = os.path.join(pkg_desc, 'config', 'controller_manager.yaml')
    arm_controller_file = os.path.join(pkg_desc, 'config', 'arm_controller.yaml')
    gripper_controller_file = os.path.join(pkg_desc, 'config', 'gripper_controller.yaml')


    # ── MoveIt config ─────────────────────────────────────────────────────────
    moveit_config = (
        MoveItConfigsBuilder("robbie", package_name="my_robot_moveit_config")
        .robot_description(
            file_path="config/robbie.urdf.xacro",
            mappings={
                'initial_positions_file': initial_positions_file,
                'controller_manager_file': controller_manager_file,
                'arm_controller_file': arm_controller_file,
                'gripper_controller_file': gripper_controller_file,
            }
        )
        .robot_description_semantic(file_path="config/robbie.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .to_moveit_configs()
    )


    controllers_yaml = load_yaml('my_robot_moveit_config', 'config/moveit_controllers.yaml')

    # 1. Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items()
    )

    # 2. Robot State Publisher
# Update your robot_state_publisher node (around line 60 in your launch file):
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': moveit_config.robot_description['robot_description'],
            'use_sim_time': True,
            'publish_frequency': 100.0,  # Add this
        }],
        output='screen'
    )

    # 3. Spawn robot in Gazebo
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'robbie'],
        output='screen'
    )

    # 4. ros_gz_bridge — clock only
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
        output='screen'
    )

    # 5. Joint State Broadcaster
    # Replace your joint_state_broadcaster_spawner with:
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    # 8. Move Group — starts after all controllers are up
    move_group_node = TimerAction(
        period=28.0,  # ✅ FIX: after gripper_controller at 21s
        actions=[
            Node(
                package='moveit_ros_move_group',
                executable='move_group',
                output='screen',
                parameters=[
                    moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                    moveit_config.joint_limits,
                    moveit_config.trajectory_execution,
                    moveit_config.planning_pipelines,
                    moveit_config.pilz_cartesian_limits,
                    controllers_yaml,  # ✅ last so it wins over trajectory_execution
                    {'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager'},
                    {'publish_robot_description': True},
                    {'publish_robot_description_semantic': True},
                    sim_time,
                ]
                # remappings=[
                #     ('/controller_manager/list_controllers', '/controller_manager/list_controllers'),
                #     ('/arm_controller/follow_joint_trajectory', '/arm_controller/follow_joint_trajectory'),
                # ]
            )
        ]
    )

    # 9. RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', str(moveit_config.package_path / 'config/moveit.rviz')],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            sim_time,
        ]
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_robot,
        bridge,
        TimerAction(
            period=5.0,
            actions=[joint_state_broadcaster_spawner] # 2. broadcaster
        ),
        TimerAction(
            period=7.0,
            actions=[arm_controller_spawner]          # 3. arm
        ),
        TimerAction(
            period=9.0,
            actions=[gripper_controller_spawner]      # 4. gripper
        ),
        move_group_node,   # already TimerAction(period=28s), no extra wrap needed
        rviz_node,
    ])