#!/usr/bin/env python3
import cv2
import time
import numpy as np
from ultralytics import YOLO
import ArducamDepthCamera as ac
from ArducamDepthCamera.ArducamDepthCamera import FrameType, Connection

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from trash_camera.msg import TrashInfo

TRASH_CLASSES = [
    'Aluminium foil', 'Battery', 'Aluminium blister pack', 'Carded blister pack', 'Other plastic bottle',
    'Clear plastic bottle', 'Glass bottle', 'Plastic bottle cap', 'Metal bottle cap', 'Broken glass',
    'Food Can', 'Aerosol', 'Drink can', 'Toilet tube', 'Other carton', 'Egg carton', 'Drink carton',
    'Corrugated carton', 'Meal carton', 'Pizza box', 'Paper cup', 'Disposable plastic cup', 'Foam cup',
    'Glass cup', 'Other plastic cup', 'Glass jar', 'Plastic lid', 'Metal lid', 'Other plastic',
    'Magazine paper', 'Tissues'
]

CORRECTION_FACTOR = 0.115
CONFIDENCE_THRESHOLD = 20
MAX_DISTANCE_MM = 4000
CAMERA_INDEX = 1  # /dev/video1

class TrashCameraPublisher(Node):
    def __init__(self):
        super().__init__('trash_camera_node')

        self.trash_pub = self.create_publisher(TrashInfo, 'trash_info', 10)
        self.model = YOLO('/home/jdamr/Downloads/best.pt')

        self.tof = ac.ArducamCamera()
        if self.tof.open(Connection.CSI, 0) != 0:
            self.get_logger().error("❌ ToF 카메라 열기 실패")
            exit()
        if self.tof.start(FrameType.DEPTH) != 0:
            self.get_logger().error("❌ ToF depth 스트림 시작 실패")
            exit()
        self.tof.setControl(ac.Control.RANGE, MAX_DISTANCE_MM)
        time.sleep(1)

        self.cap = cv2.VideoCapture(CAMERA_INDEX)
        if not self.cap.isOpened():
            self.get_logger().error(f"❌ RGB 카메라 열기 실패 (/dev/video{CAMERA_INDEX})")
            exit()
        ret, frame = self.cap.read()
        if not ret or frame is None:
            self.get_logger().error(f"❌ RGB 카메라 프레임 읽기 실패 (/dev/video{CAMERA_INDEX})")
            exit()
        self.get_logger().info(f"✅ RGB 카메라 열림: /dev/video{CAMERA_INDEX}")

        self.timer = self.create_timer(0.2, self.process_frame)

    def get_valid_depth(self):
        for _ in range(5):
            frame = self.tof.requestFrame(2000)
            if frame and isinstance(frame, ac.DepthData):
                if frame.confidence_data is not None and frame.depth_data is not None:
                    return frame
            time.sleep(0.1)
        self.get_logger().warn("⚠️ depth frame 획득 실패")
        return None

    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret or frame is None:
            return

        frame = frame[10:480, :]
        h_rgb, w_rgb = frame.shape[:2]

        depth_frame = self.get_valid_depth()
        if not depth_frame:
            self.tof.close()
            self.tof.open(Connection.CSI, 0)
            self.tof.start(FrameType.DEPTH)
            return

        depth_image = depth_frame.depth_data
        confidence_image = depth_frame.confidence_data
        h_tof, w_tof = depth_image.shape[:2]

        results = self.model.predict(frame, conf=0.2, imgsz=640)[0]  # conf 상향 및 이미지 해상도 고정

        for box in results.boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])

            padding = 0
            x1 = max(0, x1 - padding)
            y1 = max(0, y1 - padding)
            x2 = min(frame.shape[1], x2 + padding)
            y2 = min(frame.shape[0], y2 + padding)

            cx_rgb = int((x1 + x2) / 2)
            cy_rgb = int((y1 + y2) / 2)

            cx_tof = int(cx_rgb * w_tof / w_rgb)
            cy_tof = int(cy_rgb * h_tof / h_rgb)

            if 0 <= cx_tof < w_tof and 0 <= cy_tof < h_tof:
                raw_mm = depth_image[cy_tof, cx_tof]
                confidence = confidence_image[cy_tof, cx_tof]
                if confidence < CONFIDENCE_THRESHOLD:
                    continue

                corrected_cm = raw_mm * CORRECTION_FACTOR
                if corrected_cm <= 0 or corrected_cm > 200:
                    continue

                cls_id = int(box.cls[0].item())
                label = self.model.names[cls_id]
                is_trash = label in TRASH_CLASSES

                self.get_logger().info(f"YOLO 검지: {label}, 쓰레기 유무: {is_trash}")

                msg = TrashInfo()
                msg.is_trash = is_trash
                msg.distance_cm = float(corrected_cm)
                msg.center = Point(x=float(cx_rgb), y=float(cy_rgb), z=0.0)
                self.trash_pub.publish(msg)

                color = (0, 255, 0) if is_trash else (0, 0, 255)
                label_text = "Trash" if is_trash else "Not Trash"
                cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                cv2.putText(frame, f"{label_text} ({corrected_cm:.1f}cm)",
                            (x1, y1 + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

                self.get_logger().info(
                    f"🟢 검지됨: {label_text} | 거리: {corrected_cm:.1f}cm"
                )

        # cv2.imshow("RGB Camera", frame)
        # depth_vis = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX)
        # depth_vis = np.uint8(depth_vis)
        # depth_vis = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)
        # cv2.imshow("ToF Depth", depth_vis)
        # cv2.waitKey(10)

def main(args=None):
    rclpy.init(args=args)
    node = TrashCameraPublisher()
    rclpy.spin(node)
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
