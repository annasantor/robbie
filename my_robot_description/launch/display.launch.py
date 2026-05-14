import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():

    pkg = get_package_share_directory('my_robot_description')
    sdf_file = os.path.join(pkg, 'urdf', 'physical_robotic_arm.sdf')

    return LaunchDescription([

        # Start Gazebo
        ExecuteProcess(
            cmd=['gz', 'sim', '-v', '4', 'empty.sdf'],
            output='screen'
        ),

        # Spawn the robot from SDF file
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', 'physical_robotic_arm',
                '-file', sdf_file
            ],
            output='screen'
        ),

    ])