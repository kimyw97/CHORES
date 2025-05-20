from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_robot_arm_pkg',
            executable='arm_control_node',
            name='arm_control_node',
            output='screen'
        ),
        Node(
            package='my_robot_arm_pkg',
            executable='arm_control_pub_node',
            name='arm_control_pub_node',
            output='screen'
        )
    ])
