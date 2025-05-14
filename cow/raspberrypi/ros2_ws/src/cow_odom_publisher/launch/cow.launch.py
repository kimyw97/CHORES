from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    rplidar_path = FindPackageShare('rplidar_ros').find('rplidar_ros')
    rplidar_launch = os.path.join(rplidar_path, 'launch', 'rplidar_a1_launch.py')

    return LaunchDescription([
        # rplidar 실행
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rplidar_launch)
        ),
    # robot_monitoring 실행
        Node(
            package='robot_monitoring',
            executable='robot_monitor_node',
            name='robot_monitor_node',
            output='screen'
        ),

        # cmdvel_to_serial_cpp 실행
        Node(
            package='cmdvel_to_serial_cpp',
            executable='cmdvel_to_serial',
            name='cmdvel_to_serial',
            output='screen'
        ),

        # cow_odom_publisher 실행
        Node(
            package='cow_odom_publisher',
            executable='cow_odom_publisher',
            name='cow_odom_publisher',
            output='screen'
        ),
    ])

