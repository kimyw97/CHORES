#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class GoalSender(Node):
    def __init__(self, x, y):
        super().__init__('goal_sender')
        self.publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.timer = self.create_timer(1.0, self.send_goal)
        self.goal_sent = False
        self.x = float(x)
        self.y = float(y)

    def send_goal(self):
        if self.goal_sent:
            return

        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = self.x
        goal.pose.position.y = self.y
        goal.pose.orientation.w = 1.0

        self.publisher.publish(goal)
        self.get_logger().info(f'Goal sent: ({self.x}, {self.y})')
        self.goal_sent = True

def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) != 3:
        print("Usage: ros2 run jdamr200_cartographer goal_sender <x> <y>")
        return

    x = sys.argv[1]
    y = sys.argv[2]

    node = GoalSender(x, y)
    rclpy.spin_once(node, timeout_sec=2.0)  # 한 번 퍼블리시하고 종료
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
