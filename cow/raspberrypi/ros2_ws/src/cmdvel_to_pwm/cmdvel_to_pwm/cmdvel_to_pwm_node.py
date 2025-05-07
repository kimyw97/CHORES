#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial

class CmdVelToSerial(Node):
    def __init__(self):
        super().__init__('cmdvel_to_pwm_node')
        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.ser = serial.Serial('/dev/serial0', 115200, timeout=1)
        self.wheel_base = 0.2  # 바퀴 간 거리 (단위: m)
        self.max_pwm = 255     # 최대 PWM 값
        self.max_speed = 0.5   # 최대 속도 (m/s)

    def cmd_vel_callback(self, msg):
        linear = msg.linear.x
        angular = msg.angular.z
        left_speed = linear - angular * self.wheel_base / 2
        right_speed = linear + angular * self.wheel_base / 2

        # 속도를 PWM 값으로 매핑
        left_pwm = int(self.clamp(left_speed / self.max_speed * self.max_pwm))
        right_pwm = int(self.clamp(right_speed / self.max_speed * self.max_pwm))

        serial_msg = f"L:{left_pwm} R:{right_pwm}\n"
        self.ser.write(serial_msg.encode())

    def clamp(self, value):
        return max(-self.max_pwm, min(self.max_pwm, value))

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToSerial()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

