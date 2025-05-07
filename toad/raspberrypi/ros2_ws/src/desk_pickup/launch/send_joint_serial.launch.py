from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder(robot_name="your_robot_name")  # 예: "myrobot"
        .to_moveit_configs()
    )

    # RViz 설정 파일 경로 (있으면 사용, 없으면 생략 가능)
    rviz_config_path = os.path.join(
        get_package_share_directory('desk_pickup'),
        'rviz',
        'moveit2_serial_view.rviz'  # 직접 만든 rviz 설정 파일 (없으면 생략 가능)
    )

    return LaunchDescription([
        # 🟦 RViz 노드
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path],
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
                moveit_config.planning_pipelines,
                moveit_config.joint_limits,
            ],
        ),

        # 🟩 실제 IK 계산 + 시리얼 노드
        Node(
            package='desk_pickup',
            executable='send_joint_serial_sub_node',
            name='send_joint_serial_node',
            output='screen',
            emulate_tty=True,
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
            ],
        ),
    ])
