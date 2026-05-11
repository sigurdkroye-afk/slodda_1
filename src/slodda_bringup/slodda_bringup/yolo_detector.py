#!/usr/bin/env python3
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
        self.last_inference_time = self.get_clock().now()
        self.last_result = None

        self.create_subscription(Image, '/camera/image_raw', self.image_cb, qos_profile_sensor_data)
        self.det_pub = self.create_publisher(Detection2DArray, '/yolo/detections', 10)
        self.img_pub = self.create_publisher(Image, '/yolo/image', 10)

        self.get_logger().info('YoloDetector klar — venter på /camera/image_raw')

    def image_cb(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'cv_bridge: {e}')
            return

        now = self.get_clock().now()
        if (now - self.last_inference_time).nanoseconds >= 200_000_000:  # 5 Hz
            self.last_inference_time = now
            results = self.model.predict(frame, classes=[77], conf=0.15, verbose=False, device='cpu')
            self.last_result = results[0]

            arr = Detection2DArray()
            arr.header = msg.header
            for box in self.last_result.boxes:
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

        annotated = self.last_result.plot() if self.last_result is not None else frame
        try:
            self.img_pub.publish(self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8'))
        except Exception as e:
            self.get_logger().warn(f'annotert bilde: {e}')


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
