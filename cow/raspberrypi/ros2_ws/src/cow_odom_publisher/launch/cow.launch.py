from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir, FindExecutable
import os

def generate_launch_description():
    # rplidar_ros의 rplidar.launch.py 포함
    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                os.path.join(
                    os.getenv('AMENT_PREFIX_PATH').split(':')[0],
                    'share', 'rplidar_ros', 'launch', 'rplidar.launch.py'
                )
            )
        ])
    )

    return LaunchDescription([
        # LiDAR 실행
        rplidar_launch,

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

