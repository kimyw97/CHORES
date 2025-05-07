from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cmdvel_to_pwm',
            executable='cmdvel_to_pwm',
            name='cmdvel_to_pwm_node',
            output='screen',
            parameters=[],
        )
    ])
