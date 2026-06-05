from launch import LaunchDescription
from launch.actions import TimerAction
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

    initial_positions_file = os.path.join(
        get_package_share_directory('my_robot_moveit_config'),
        'config', 'initial_positions.yaml'
    )

    pkg_desc = get_package_share_directory('my_robot_description')
    controller_file = os.path.expanduser('~/ros2_controllers.yaml')

    moveit_config = (
        MoveItConfigsBuilder("robbie", package_name="my_robot_moveit_config")
        .robot_description(
            file_path="config/robbie.urdf.xacro",
            mappings={
                'initial_positions_file': initial_positions_file,
                'controller_file': controller_file,
                'use_real_hardware': 'true',    # ← key difference
            }
        )
        .robot_description_semantic(file_path="config/robbie.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .to_moveit_configs()
    )

    controllers_yaml = load_yaml('my_robot_moveit_config', 'config/moveit_controllers.yaml')

    # 1. Robot State Publisher — no sim time
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': moveit_config.robot_description['robot_description'],
            'use_sim_time': False,
            'publish_frequency': 100.0,
        }],
        output='screen'
    )

    # 2. ros2_control node — loads your hardware interface plugin
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': moveit_config.robot_description['robot_description']},
            controller_file,
        ],
        output='screen'
    )

    # 3. Controllers
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller_safe', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    # 4. Move Group
    move_group_node = TimerAction(
        period=10.0,
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
                    controllers_yaml,
                    {'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager'},
                    {'publish_robot_description': True},
                    {'publish_robot_description_semantic': True},
                    {'use_sim_time': False},
                ]
            )
        ]
    )

    # 5. RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', str(moveit_config.package_path / 'config/moveit.rviz')],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {'use_sim_time': False},
        ]
    )

    return LaunchDescription([
        robot_state_publisher,
        ros2_control_node,
        TimerAction(period=3.0,  actions=[joint_state_broadcaster_spawner]),
        TimerAction(period=5.0,  actions=[arm_controller_spawner]),
        TimerAction(period=7.0,  actions=[gripper_controller_spawner]),
        move_group_node,
        rviz_node,
    ])