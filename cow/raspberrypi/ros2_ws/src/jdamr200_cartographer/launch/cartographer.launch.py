#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # 기본 LaunchConfiguration
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    resolution = LaunchConfiguration('resolution', default='0.05')
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')

    # 고정 문자열로 경로 설정 (여기 수정 포인트!)
    config_dir_path = os.path.join(
        get_package_share_directory('jdamr200_cartographer'), 'config'
    )
    configuration_basename_str = 'jdamr200_lidar.lua'

    # 다른 launch 파일 경로
    rplidar_launch = os.path.join(
        get_package_share_directory('rplidar_ros'),
        'launch', 'rplidar_a1_launch.py'
    )
    occupancy_launch = os.path.join(
        get_package_share_directory('jdamr200_cartographer'),
        'launch', 'occupancy_grid.launch.py'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('resolution', default_value=resolution),
        DeclareLaunchArgument('publish_period_sec', default_value=publish_period_sec),

        # ✅ RPLIDAR 실행
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rplidar_launch),
            launch_arguments={'use_sim_time': use_sim_time}.items()
        ),

        # ✅ Static Transform 설정
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_base_laser_tf',
            arguments=[
                '0', '0', '0.18',
                '0', '0', '0', '1',
                'base_link',
                'base_laser'
            ]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom_tf',
            arguments=[
                '0', '0', '0',
                '0', '0', '0', '1',
                'map',
                'odom'
            ]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='odom_to_base_link_tf',
            arguments=[
                '0', '0', '0',
                '0', '0', '0', '1',
                'odom',
                'base_link'
            ]
        ),

        # ✅ Cartographer SLAM 노드
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[
                '-configuration_directory', config_dir_path,
                '-configuration_basename', configuration_basename_str
            ]
        ),

        # ✅ Occupancy grid 노드
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(occupancy_launch),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'resolution': resolution,
                'publish_period_sec': publish_period_sec
            }.items()
        ),

        #✅ RViz 실행
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
        )
    ])
