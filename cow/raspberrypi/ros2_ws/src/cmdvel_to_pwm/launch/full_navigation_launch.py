#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 경로 설정
    cmdvel_pkg = get_package_share_directory('cmdvel_to_pwm')
    carto_pkg = get_package_share_directory('jdamr200_cartographer')
    rplidar_pkg = get_package_share_directory('rplidar_ros')

    carto_config_dir = os.path.join(carto_pkg, 'config')
    carto_basename = 'cartographer.lua'
    param_file = os.path.join(cmdvel_pkg, 'config', 'nav2_params.yaml')
    rviz_config_file = os.path.join(cmdvel_pkg, 'config', 'slam_config.rviz')

    rplidar_launch = os.path.join(rplidar_pkg, 'launch', 'rplidar.launch.py')
    occupancy_launch = os.path.join(carto_pkg, 'launch', 'occupancy_grid.launch.py')

    # 런치 인자
    use_sim_time = LaunchConfiguration('use_sim_time')
    resolution = LaunchConfiguration('resolution')
    publish_period_sec = LaunchConfiguration('publish_period_sec')
    map_file = LaunchConfiguration('map')

    return LaunchDescription([
        # 인자 선언
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('resolution', default_value='0.05'),
        DeclareLaunchArgument('publish_period_sec', default_value='1.0'),
        DeclareLaunchArgument('map', default_value=os.path.join(cmdvel_pkg, 'map', 'map.yaml')),

        # ✅ RPLIDAR 실행
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rplidar_launch),
            launch_arguments={'use_sim_time': use_sim_time}.items()
        ),

        # ✅ Static TF 설정
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_base_laser_tf',
            arguments=['0', '0', '0.18', '0', '0', '0', '1', 'base_link', 'base_laser']
        ),

        # ✅ Cartographer SLAM
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[
                '-configuration_directory', carto_config_dir,
                '-configuration_basename', carto_basename
            ],
            remappings=[('/scan', '/scan')]
        ),

        # ✅ Occupancy Grid 노드 실행
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(occupancy_launch),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'resolution': resolution,
                'publish_period_sec': publish_period_sec
            }.items()
        ),

        # ✅ map_server
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'yaml_filename': map_file}
            ]
        ),

        # ✅ AMCL Localization
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[param_file]
        ),

        # ✅ Nav2 Core Nodes
        Node(package='nav2_controller', executable='controller_server', output='screen', parameters=[param_file]),
        Node(package='nav2_planner', executable='planner_server', output='screen', parameters=[param_file]),
        Node(package='nav2_bt_navigator', executable='bt_navigator', output='screen', parameters=[param_file]),

        # ✅ Lifecycle Manager
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': [
                    'map_server',
                    'amcl',
                    'controller_server',
                    'planner_server',
                    'bt_navigator'
                ]
            }]
        ),

        # ✅ 모터 제어 노드
        Node(
            package='cmdvel_to_pwm',
            executable='cmdvel_to_pwm_node',
            name='cmdvel_to_pwm_node',
            output='screen'
        ),

        # ✅ RViz 실행
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file]
        )
    ])
