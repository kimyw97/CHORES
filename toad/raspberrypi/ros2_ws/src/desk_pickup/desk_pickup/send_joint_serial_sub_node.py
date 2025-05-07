# desk_pickup/send_joint_serial_sub_node.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

from moveit_commander import MoveGroupCommander, PlanningSceneInterface, RobotCommander
import geometry_msgs.msg
import serial
import time

class IKSerialNode(Node):
    def __init__(self):
        super().__init__('ik_serial_node')

        # MoveIt2 초기화
        self.robot = RobotCommander()
        self.move_group = MoveGroupCommander("your_arm_group")  # 네 로봇에 맞게 수정
        self.move_group.set_planning_time(5.0)

        # 시리얼 포트 설정
        try:
            self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
            time.sleep(2)
            self.get_logger().info("✅ Serial port opened.")
        except:
            self.get_logger().error("❌ Failed to open serial port.")
            self.ser = None

        # Subscriber 설정
        self.subscription = self.create_subscription(
            Float32,
            '/target_distance',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        distance = msg.data
        self.get_logger().info(f"📩 Received target distance: {distance:.2f} m")

        target_pose = geometry_msgs.msg.Pose()
        target_pose.position.x = distance
        target_pose.position.y = 0.0
        target_pose.position.z = 0.3  # 고정 높이
        target_pose.orientation.w = 1.0  # Quaternion default

        self.move_group.set_pose_target(target_pose)

        plan = self.move_group.plan()
        if plan[0]:  # planning 성공
            joint_positions = plan[1].joint_trajectory.points[-1].positions
            joint_string = ''.join([f"{p:+.2f}" for p in joint_positions])
            self.get_logger().info(f"🦾 Joint Angles: {joint_string}")

            if self.ser:
                self.ser.write((joint_string + '\n').encode())
                self.get_logger().info("📤 Sent to serial.")
        else:
            self.get_logger().warn("⚠️ IK Planning failed.")

def main(args=None):
    rclpy.init(args=args)
    node = IKSerialNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


# class SerialSender(Node):
#     def __init__(self):
#         super().__init__('send_joint_serial_sub_node')

#         # 시리얼 포트 설정
#         self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)  # 포트랑 속도 맞게 수정
#         self.get_logger().info("Serial port opened.")

#         # 서브스크라이버 생성
#         self.subscription = self.create_subscription(
#             Int32,
#             '/pick_n_place',
#             self.listener_callback,
#             10)

#         # 미리 준비된 조인트 값
#         self.joint_command = "900450300200100150\n"

#     def listener_callback(self, msg):
#         if msg.data == 1:
#             self.get_logger().info("Received 1 → Sending Joint Command!")
#             self.ser.write(self.joint_command.encode())
#         else:
#             self.get_logger().info(f"Received {msg.data} → Waiting...")

#     def destroy_node(self):
#         self.ser.close()
#         super().destroy_node()

# def main(args=None):
#     rclpy.init(args=args)
#     node = SerialSender()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()
