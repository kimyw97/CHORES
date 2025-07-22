import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
import time

class AccelerationEstimator(Node):
    def __init__(self):
        super().__init__('acceleration_estimator')

        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.cmd_time = None
        self.cmd_vel = None
        self.last_vel = None
        self.last_ang = None
        self.last_time = None

        self.accel_samples = []
        self.angular_samples = []

    def cmd_vel_callback(self, msg):
        self.cmd_time = time.time()
        self.cmd_vel = msg

    def odom_callback(self, msg):
        if self.cmd_time is None:
            return

        now = time.time()
        linear_x = msg.twist.twist.linear.x
        angular_z = msg.twist.twist.angular.z

        if self.last_vel is not None and self.last_time is not None:
            dt = now - self.last_time
            if dt < 0.01:
                return

            dv = linear_x - self.last_vel
            accel = dv / dt
            self.accel_samples.append(accel)

            dw = angular_z - self.last_ang
            ang_accel = dw / dt
            self.angular_samples.append(ang_accel)

            if len(self.accel_samples) >= 50:
                max_accel = np.max(np.abs(self.accel_samples))
                max_ang = np.max(np.abs(self.angular_samples))
                self.get_logger().info(f"Estimated max linear accel: {max_accel:.2f} m/s^2")
                self.get_logger().info(f"Estimated max angular accel: {max_ang:.2f} rad/s^2")
                self.accel_samples.clear()
                self.angular_samples.clear()

        self.last_vel = linear_x
        self.last_ang = angular_z
        self.last_time = now

def main(args=None):
    rclpy.init(args=args)
    node = AccelerationEstimator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
