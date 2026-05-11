#!/usr/bin/env python3
import os
import queue
import threading
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge
from ultralytics import YOLO


class YoloDetector(Node):
    def __init__(self):
        super().__init__('yolo_detector')
        self.bridge = CvBridge()
        self.model = YOLO('yolov8n.pt')

        # Non-blocking inference: frames queued here, worker thread processes them.
        # maxsize=1 drops stale frames so YOLO always sees the freshest image.
        self._frame_queue = queue.Queue(maxsize=1)
        self._last_result = None
        self._last_header = None

        self.create_subscription(Image, '/camera/image_raw', self.image_cb, qos_profile_sensor_data)
        self.det_pub = self.create_publisher(Detection2DArray, '/yolo/detections', 10)
        self.img_pub = self.create_publisher(Image, '/yolo/image', 10)

        threading.Thread(target=self._infer_loop, daemon=True).start()
        self.get_logger().info('YoloDetector klar — venter på /camera/image_raw')

    def image_cb(self, msg: Image):
        """Fast callback: convert and queue frame, never block."""
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'cv_bridge: {e}')
            return
        try:
            self._frame_queue.put_nowait((frame, msg.header))
        except queue.Full:
            pass  # drop stale frame — worker is still busy with previous

    def _infer_loop(self):
        """Background worker: YOLO inference without blocking the ROS2 executor."""
        os.nice(10)  # lower priority so EKF/Nav2 preempt during inference
        while True:
            try:
                frame, header = self._frame_queue.get(timeout=1.0)
            except queue.Empty:
                continue

            results = self.model.predict(frame, classes=[77], conf=0.15, verbose=False, device='cpu')
            result = results[0]
            self._last_result = result
            self._last_header = header

            arr = Detection2DArray()
            arr.header = header
            for box in result.boxes:
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                det = Detection2D()
                det.bbox.center.position.x = (x1 + x2) / 2.0
                det.bbox.center.position.y = (y1 + y2) / 2.0
                det.bbox.size_x = x2 - x1
                det.bbox.size_y = y2 - y1
                hyp = ObjectHypothesisWithPose()
                hyp.hypothesis.class_id = '77'
                hyp.hypothesis.score = float(box.conf[0])
                det.results.append(hyp)
                arr.detections.append(det)

            self.det_pub.publish(arr)

            if arr.detections:
                self.get_logger().info(f'Detected {len(arr.detections)} teddy bear(s)')

            try:
                annotated = result.plot()
                self.img_pub.publish(self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8'))
            except Exception:
                pass

            # Brief cooldown so Pi4 can catch up on EKF/Nav2 between inferences
            time.sleep(2.0)


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
