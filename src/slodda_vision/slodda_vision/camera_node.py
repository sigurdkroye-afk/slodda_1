import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from picamera2 import Picamera2
from libcamera import controls
import time


class CameraNode(Node):
    def __init__(self):
        super().__init__("camera_node")
        self.pub = self.create_publisher(Image, "/camera/image_raw", 10)

        self.picam2 = Picamera2()
        self.picam2.configure(
            self.picam2.create_preview_configuration(
                main={"format": "RGB888", "size": (640, 480)}
            )
        )
        self.picam2.start()
        time.sleep(3)
        self.picam2.set_controls({"AwbEnable": True, "AwbMode": controls.AwbModeEnum.Tungsten})
        time.sleep(2)

        self.get_logger().info("Kamera klar — publiserer paa /camera/image_raw")
        self.create_timer(0.1, self.publish_frame)

    def publish_frame(self):
        frame = self.picam2.capture_array()
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera"
        msg.height = frame.shape[0]
        msg.width = frame.shape[1]
        msg.encoding = "rgb8"
        msg.is_bigendian = False
        msg.step = frame.shape[1] * 3
        msg.data = frame.tobytes()
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.picam2.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
