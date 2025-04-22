import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial

# 포트가 없을 때 사용될 더미 시리얼 클래스
class DummySerial:
    def write(self, data):
        print(f"[DummySerial] 전송됨: {data.decode().strip()}")

    def close(self):
        print("[DummySerial] 닫힘")

class TwistToSerial(Node):
    def __init__(self):
        super().__init__('twist_to_serial_node')

        # /cmd_vel 토픽 구독자 생성
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.listener_callback,
            10
        )

        # 아두이노 시리얼 포트 연결 설정
        try:
            self.serial_port = serial.Serial(
                '/dev/ttyUSB0',
                baudrate=115200,
                timeout=1
            )
            self.get_logger().info('Arduino와 시리얼 연결 성공')
        except serial.SerialException:
            self.get_logger().warn('시리얼 포트 연결 실패, DummySerial 사용')
            self.serial_port = DummySerial()

    def listener_callback(self, msg: Twist):
        # /cmd_vel에서 선속도, 회전속도 추출
        linear = msg.linear.x
        angular = msg.angular.z

        # 좌/우 바퀴 속도 계산
        left_speed = linear - angular
        right_speed = linear + angular

        # 속도를 100단위 pwm값으로 변환
        left_pwm = int(left_speed * 100)
        right_pwm = int(right_speed * 100)

        # pwm값 제한
        left_pwm = max(min(left_pwm, 255), -255)
        right_pwm = max(min(right_pwm, 255), -255)

        # 시리얼로 보낼 문자열 포맷
        command = f"L{left_pwm}R{right_pwm}\n"

        # 로그 출력
        self.get_logger().info(f"- 시리얼 전송: {command.strip()}")

        # 시리얼 전송
        try:
            self.serial_port.write(command.encode())
        except Exception as e:
            self.get_logger().error(f"시리얼 전송 오류: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = TwistToSerial()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.serial_port.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
