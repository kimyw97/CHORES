from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 퍼블리셔: WASD 키 입력 → /cmd_vel 퍼블리시
        Node(
            package='teleop_bridge',                  # 패키지 이름
            executable='teleop_wasd_publisher',       # 퍼블리셔 실행파일
            name='teleop_wasd_publisher_node',
            output='screen',
            prefix='xterm -hold -e',
        ),

        # 서브스크라이버: /cmd_vel → 시리얼로 모터 제어
        Node(
            package='teleop_bridge',                  # 동일한 패키지라면 그대로
            executable='teleop_to_serial',            # 서브스크라이버 실행파일
            name='teleop_to_serial_node',
            output='screen',
        )
    ])
