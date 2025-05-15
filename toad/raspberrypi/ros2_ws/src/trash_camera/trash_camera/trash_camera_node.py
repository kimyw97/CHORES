import cv2
import time
import numpy as np
from ultralytics import YOLO
import ArducamDepthCamera
from ArducamDepthCamera.ArducamDepthCamera import FrameType, Connection

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32
from geometry_msgs.msg import Point

TRASH_CLASSES = [
    'Aluminium foil', 'Battery', 'Aluminium blister pack', 'Carded blister pack', 'Other plastic bottle',
    'Clear plastic bottle', 'Glass bottle', 'Plastic bottle cap', 'Metal bottle cap', 'Broken glass',
    'Food Can', 'Aerosol', 'Drink can', 'Toilet tube', 'Other carton', 'Egg carton', 'Drink carton',
    'Corrugated carton', 'Meal carton', 'Pizza box', 'Paper cup', 'Disposable plastic cup', 'Foam cup',
    'Glass cup', 'Other plastic cup', 'Glass jar', 'Plastic lid', 'Metal lid', 'Other plastic',
    'Magazine paper', 'Tissues'
]

CORRECTION_FACTOR = 0.11

class TrashCameraPublisher(Node):
    def __init__(self):
        super().__init__('trash_camera_node')

        self.trash_pub = self.create_publisher(Bool, 'is_trash', 10)
        self.dist_pub = self.create_publisher(Float32, 'trash_distance', 10)
        self.center_pub = self.create_publisher(Point, 'trash_center', 10)

        self.model = YOLO('/home/jdamr/Downloads/litter-detection-master/runs/detect/train/yolov8s_100epochs/weights/best.pt')

        self.tof = ArducamDepthCamera.ArducamCamera()
        if self.tof.open(Connection.CSI, 0) != 0:
            self.get_logger().error("❌ ToF 카메라 연결 실패")
            exit()
        self.tof.start(FrameType.DEPTH)
        time.sleep(1)

        self.cap = self.find_camera()
        if self.cap is None:
            self.get_logger().error("❌ RGB 카메라 없음")
            exit()

        self.timer = self.create_timer(0.2, self.process_frame)

    def find_camera(self):
        for i in range(10):
            cap = cv2.VideoCapture(i)
            if cap.isOpened():
                ret, frame = cap.read()
                if ret:
                    self.get_logger().info(f"✅ RGB 카메라 열림: /dev/video{i}")
                    return cap
                cap.release()
        return None

    def get_valid_depth(self):
        for _ in range(5):
            frame = self.tof.requestFrame(1000)
            if frame and hasattr(frame, "depth_data") and frame.depth_data is not None:
                return frame
            time.sleep(0.1)
        return None

    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret or frame is None:
            return

        frame = frame[10:480, :]  # ROI 적용 (상단 100픽셀 제거)
        roi_height = frame.shape[0]

        depth_data = self.get_valid_depth()
        if not depth_data:
            self.tof.close()
            self.tof.open(Connection.CSI, 0)
            self.tof.start(FrameType.DEPTH)
            return

        depth_image = depth_data.depth_data

        results = self.model.predict(frame, conf=0.5)[0]
        for box in results.boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            confidence = box.conf[0].item()
            cls_id = int(box.cls[0].item())
            label = self.model.names[cls_id]

            cx_rgb = int((x1 + x2) / 2)
            cy_rgb = int((y1 + y2) / 2)
            h, w = depth_image.shape[:2]
            cx_tof = int(cx_rgb * w / 640)
            cy_tof = int(cy_rgb * h / roi_height)

            if 0 <= cx_tof < w and 0 <= cy_tof < h:
                raw_mm = depth_image[cy_tof, cx_tof]
                corrected_cm = raw_mm * CORRECTION_FACTOR

                if corrected_cm <= 0 or corrected_cm > 30:
                    continue

                is_trash = label in TRASH_CLASSES

                self.trash_pub.publish(Bool(data=is_trash))
                self.dist_pub.publish(Float32(data=corrected_cm))
                self.center_pub.publish(Point(x=float(cx_rgb), y=float(cy_rgb), z=0.0))

                self.get_logger().info(f"🟢 감지됨: {label} {confidence:.2f} | 거리: {corrected_cm:.1f}cm | 쓰레기: {is_trash}")
                break

def main(args=None):
    rclpy.init(args=args)
    node = TrashCameraPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
