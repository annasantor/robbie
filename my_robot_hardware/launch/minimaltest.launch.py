from launch import LaunchDescription
from launch.actions import TimerAction
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    pkg_desc = get_package_share_directory('my_robot_description')
    controller_file = os.path.join(pkg_desc, 'config', 'ros2_controllers.yaml')
    urdf_path = os.path.join(pkg_desc, 'urdf', 'robbie.urdf.xacro')

    # Symlink approach — same as working Gazebo launch
    # ln -s ~/ros2_ws/src/my_robot_description/config/ros2_controllers.yaml \
    #       ~/ros2_controllers.yaml
    controllers_yaml = os.path.expanduser('~/ros2_controllers.yaml')

    robot_description = ParameterValue(
        Command([
            'xacro ', urdf_path,
            ' use_real_hardware:=true',
            ' controller_file:=', controller_file,
        ]),
        value_type=str
    )

    return LaunchDescription([

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'publish_frequency': 50.0,
            }],
        ),

        Node(
            package='controller_manager',
            executable='ros2_control_node',
            output='screen',
            parameters=[
                {'robot_description': robot_description},
                controllers_yaml,
            ],
        ),

        TimerAction(
            period=15.0,
            actions=[
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=[
                        'joint_state_broadcaster',
                        '--controller-manager', '/controller_manager',
                    ],
                    output='screen',
                ),
            ]
        ),

        TimerAction(
            period=25.0,
            actions=[
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=[
                        'arm_controller',
                        '--controller-manager', '/controller_manager',
                    ],
                    output='screen',
                ),
            ]
        ),

        TimerAction(
            period=35.0,
            actions=[
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=[
                        'gripper_controller',
                        '--controller-manager', '/controller_manager',
                    ],
                    output='screen',
                ),
            ]
        ),

    ])