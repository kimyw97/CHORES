import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty

class TeleopWASDPublisher(Node):
    def __init__(self):
        super().__init__('teleop_wasd_publisher')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.settings = termios.tcgetattr(sys.stdin)

        # 초기 속도 설정
        self.speed = 0.5
        self.speed_step = 0.1
        self.min_speed = 0.1
        self.max_speed = 2.0

        self.get_logger().info("WASD 키로 조작하세요")
        self.get_logger().info("w: 전진, s: 후진, a: 좌회전, d: 우회전")
        self.get_logger().info("+: 속도 증가, -: 속도 감소, q: 종료")
        self.run()

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run(self):
        try:
            while True:
                key = self.get_key()
                msg = Twist()

                if key == 'w':
                    msg.linear.x = self.speed
                elif key == 's':
                    msg.linear.x = -self.speed
                elif key == 'a':
                    msg.angular.z = self.speed
                elif key == 'd':
                    msg.angular.z = -self.speed
                elif key == '+':
                    self.speed = min(self.speed + self.speed_step, self.max_speed)
                    self.get_logger().info(f"속도 증가: {self.speed:.1f}")
                    continue  # 속도만 조절, 메시지는 보내지 않음
                elif key == '-':
                    self.speed = max(self.speed - self.speed_step, self.min_speed)
                    self.get_logger().info(f"속도 감소: {self.speed:.1f}")
                    continue
                elif key == 'q':
                    self.get_logger().info("종료합니다.")
                    break

                self.publisher_.publish(msg)
                self.get_logger().info(f"발행: 선속도={msg.linear.x}, 각속도={msg.angular.z} (속도 설정: {self.speed:.1f})")

        except Exception as e:
            self.get_logger().error(f"에러 발생: {e}")
        finally:
            # 종료 시 정지 명령
            msg = Twist()
            self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TeleopWASDPublisher()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
