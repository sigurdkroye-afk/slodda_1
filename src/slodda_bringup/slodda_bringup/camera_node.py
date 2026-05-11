import cv2
import time
import threading
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from picamera2 import Picamera2


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        self._pub = self.create_publisher(Image, '/camera/image_raw', qos_profile_sensor_data)
        self._frame = None
        self._lock = threading.Lock()

        self._cam = Picamera2()
        self._cam.configure(
            self._cam.create_video_configuration(
                main={'format': 'XBGR8888', 'size': (640, 480)}
            )
        )
        self._cam.start()
        time.sleep(2)

        threading.Thread(target=self._capture_loop, daemon=True).start()
        self.get_logger().info('Camera ready — publishing on /camera/image_raw at 5 Hz')
        self.create_timer(0.2, self._publish)

    def _capture_loop(self):
        while True:
            raw = self._cam.capture_array('main')
            bgr = cv2.cvtColor(raw, cv2.COLOR_BGRA2BGR)
            with self._lock:
                self._frame = bgr
            time.sleep(0.2)

    def _publish(self):
        with self._lock:
            frame = self._frame
        if frame is None:
            return
        h, w = frame.shape[:2]
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_link'
        msg.height = h
        msg.width = w
        msg.encoding = 'bgr8'
        msg.is_bigendian = False
        msg.step = w * 3
        msg.data = frame.tobytes()
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._cam.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
