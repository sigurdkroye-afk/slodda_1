import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import numpy as np
import time

from picamera2 import Picamera2
from ultralytics import YOLO


class YoloDetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_detector')
        self.get_logger().info('YOLO Detector node starting...')

        self.bridge = CvBridge()

        # Publishers
        self.image_pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/detected_objects', 10)

        # YOLO og kamera
        self.model = YOLO('/home/slodda1/yolov8n.pt')
        self.cam = Picamera2()

        # Preview config (640x640 RGB er bra for YOLO + ROS)
        config = self.cam.create_preview_configuration(
            main={"size": (640, 640), "format": "RGB888"}
        )
        self.cam.configure(config)
        self.cam.start()

        # Timer for deteksjon (ca 2 Hz)
        self.timer = self.create_timer(0.5, self.detect_and_publish)

        self.get_logger().info('YOLO + Picamera2 initialized successfully!')

    def detect_and_publish(self):
        try:
            # Ta bilde som numpy array (RGB)
            frame = self.cam.capture_array("main")   # shape: (640, 640, 3)

            # Kjør YOLO (kun teddybjørn = class 77)
            results = self.model(frame, classes=[77], verbose=False, conf=0.4)

            # Publiser rå bildet til ROS (konverter til ROS Image msg)
            ros_image = self.bridge.cv2_to_imgmsg(frame, encoding="rgb8")
            ros_image.header.stamp = self.get_clock().now().to_msg()
            ros_image.header.frame_id = "camera_link"
            self.image_pub.publish(ros_image)

            # Bygg MarkerArray for visualisering i RViz
            marker_array = MarkerArray()
            if results[0].boxes:
                for i, box in enumerate(results[0].boxes):
                    conf = float(box.conf[0])
                    cls = int(box.cls[0])
                    xyxy = box.xyxy[0].cpu().numpy()  # [x1, y1, x2, y2]

                    marker = Marker()
                    marker.header.frame_id = "camera_link"
                    marker.header.stamp = self.get_clock().now().to_msg()
                    marker.ns = "teddy"
                    marker.id = i
                    marker.type = Marker.CUBE
                    marker.action = Marker.ADD

                    # Plasser marker i bilde-koordinater (midlertidig – du kan senere gjøre til 3D)
                    center_x = (xyxy[0] + xyxy[2]) / 2
                    center_y = (xyxy[1] + xyxy[3]) / 2
                    marker.pose.position = Point(x=center_x/100.0, y=center_y/100.0, z=0.5)  # dummy z
                    marker.scale.x = (xyxy[2]-xyxy[0])/100.0
                    marker.scale.y = (xyxy[3]-xyxy[1])/100.0
                    marker.scale.z = 0.1
                    marker.color.r = 1.0
                    marker.color.g = 0.0
                    marker.color.b = 1.0
                    marker.color.a = 0.7

                    marker_array.markers.append(marker)

                    self.get_logger().info(f'Teddy funnet! Conf: {conf:.2f} BBox: {xyxy}')

            self.marker_pub.publish(marker_array)

        except Exception as e:
            self.get_logger().error(f'Feil i detect_and_publish: {e}')

    def destroy_node(self):
        self.cam.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
