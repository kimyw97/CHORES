from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    urdf_path = '/home/kimyw/vscode/CHORES/cow_HW/cow.urdf.xacro'

    return LaunchDescription([
        # Joint State Publisher (no GUI)
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen'
        ),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                {
                    'robot_description': Command(['xacro ', urdf_path]),
                    'use_sim_time': False,
                    'publish_frequency': 10.0,
                }
            ]
        )
    ])
