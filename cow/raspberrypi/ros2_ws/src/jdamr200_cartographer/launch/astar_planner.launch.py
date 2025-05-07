from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='jdamr200_cartographer',
            executable='astar_path_planner.py',
            name='astar_path_planner',
            output='screen'
        )
    ])

