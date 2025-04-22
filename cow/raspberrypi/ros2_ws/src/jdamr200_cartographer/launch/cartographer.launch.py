import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    config_dir = LaunchConfiguration('config_dir', default=os.path.join(
        get_package_share_directory('jdamr200_cartographer'), 'config'))
    configuration_basename = LaunchConfiguration('configuration_basename', default='jdamr200_lidar.lua')
    resolution = LaunchConfiguration('resolution', default='0.05')
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')

    rplidar_launch = os.path.join(
        get_package_share_directory('rplidar_ros'),
        'launch', 'rplidar.launch.py')

    occupancy_launch = os.path.join(
        get_package_share_directory('jdamr200_cartographer'),
        'launch', 'occupancy_grid.launch.py')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('config_dir', default_value=config_dir),
        DeclareLaunchArgument('configuration_basename', default_value=configuration_basename),
        DeclareLaunchArgument('resolution', default_value=resolution),
        DeclareLaunchArgument('publish_period_sec', default_value=publish_period_sec),

        # ✅ RPLIDAR 실행
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rplidar_launch),
            launch_arguments={'use_sim_time': use_sim_time}.items()
        ),

        # ✅ 필수 TF 설정
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0.18', '0', '0', '0', 'base_link', 'base_laser'],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
        ),

        # ✅ Cartographer SLAM 노드
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[
                '-configuration_directory', config_dir,
                '-configuration_basename', configuration_basename
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

        # ✅ RViz 실행
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
        )
    ])
