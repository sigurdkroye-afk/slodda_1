import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Empty
from geometry_msgs.msg import Twist
from action_msgs.srv import CancelGoal
from nav2_msgs.srv import ClearEntireCostmap
from std_srvs.srv import Trigger
from cv_bridge import CvBridge

from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QSizePolicy,
)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QObject
from PyQt5.QtGui import QImage, QPixmap


class _Signals(QObject):
    image_ready = pyqtSignal(QImage)


class _Node(Node):
    def __init__(self, sig):
        super().__init__('hardware_panel')
        self._sig = sig
        self._bridge = CvBridge()
        self._pub_approve = self.create_publisher(Bool, '/mission/approve', 10)
        self._pub_home = self.create_publisher(Empty, '/mission/return_home', 10)
        self.create_subscription(Image, '/camera/image_raw', self._img_cb, qos_profile_sensor_data)
        self._zero_vel_pub = self.create_publisher(Twist, '/cmd_vel', 1)
        self._cancel_nav_svc = self.create_client(
            CancelGoal, '/navigate_to_pose/_action/cancel_goal')
        self._clear_local = self.create_client(
            ClearEntireCostmap, '/local_costmap/clear_entirely_local_costmap')
        self._clear_global = self.create_client(
            ClearEntireCostmap, '/global_costmap/clear_entirely_global_costmap')
        self._arm_stop_svc = self.create_client(Trigger, '/arm/stop')

    def _img_cb(self, msg):
        try:
            rgb = self._bridge.imgmsg_to_cv2(msg, 'rgb8')
            h, w = rgb.shape[:2]
            data = rgb.tobytes()
            qi = QImage(data, w, h, 3 * w, QImage.Format_RGB888).copy()
            self._sig.image_ready.emit(qi)
        except Exception as e:
            self.get_logger().warn(f'img_cb: {e}')

    def approve(self):
        self._pub_approve.publish(Bool(data=True))

    def return_home(self):
        self._pub_home.publish(Empty())

    def arm_stop(self):
        if self._arm_stop_svc.service_is_ready():
            self._arm_stop_svc.call_async(Trigger.Request())

    def cancel_nav2(self):
        self._zero_vel_pub.publish(Twist())
        if self._cancel_nav_svc.service_is_ready():
            self._cancel_nav_svc.call_async(CancelGoal.Request())
        for cli in (self._clear_local, self._clear_global):
            if cli.service_is_ready():
                cli.call_async(ClearEntireCostmap.Request())


class HardwarePanel(QWidget):
    def __init__(self, node):
        super().__init__()
        self._node = node
        self.setWindowTitle('Slodda Hardware Control')
        self.setMinimumSize(660, 520)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(5)

        self._img_label = QLabel('Waiting for camera...')
        self._img_label.setAlignment(Qt.AlignCenter)
        self._img_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self._img_label.setStyleSheet('background: #111; color: #555;')
        layout.addWidget(self._img_label, stretch=1)

        row = QHBoxLayout()
        row.setSpacing(6)

        btn_approve = QPushButton('✓ Approve')
        btn_approve.setMinimumHeight(40)
        btn_approve.setStyleSheet('background: #2a7a2a; color: white; font-size: 12pt;')
        btn_approve.clicked.connect(self._node.approve)
        row.addWidget(btn_approve)

        btn_home = QPushButton('⟵ Return Home')
        btn_home.setMinimumHeight(40)
        btn_home.setStyleSheet('background: #4a4a9a; color: white; font-size: 12pt;')
        btn_home.clicked.connect(self._node.return_home)
        row.addWidget(btn_home)

        btn_cancel = QPushButton('■ CANCEL NAV2')
        btn_cancel.setMinimumHeight(40)
        btn_cancel.setStyleSheet('background: #a02020; color: white; font-size: 12pt; font-weight: bold;')
        btn_cancel.clicked.connect(self._node.cancel_nav2)
        row.addWidget(btn_cancel)

        layout.addLayout(row)

        btn_arm_stop = QPushButton('🛑 ARM STOPP')
        btn_arm_stop.setMinimumHeight(50)
        btn_arm_stop.setStyleSheet(
            'background: #cc0000; color: white; font-size: 14pt; font-weight: bold; border: 2px solid #ff4444;'
        )
        btn_arm_stop.clicked.connect(self._node.arm_stop)
        layout.addWidget(btn_arm_stop)

    def set_image(self, qi):
        px = QPixmap.fromImage(qi)
        scaled = px.scaled(
            self._img_label.width(), self._img_label.height(),
            Qt.KeepAspectRatio, Qt.SmoothTransformation,
        )
        self._img_label.setPixmap(scaled)


def main(args=None):
    import signal
    rclpy.init(args=args)
    app = QApplication(sys.argv)
    sig = _Signals()
    node = _Node(sig)

    panel = HardwarePanel(node)
    sig.image_ready.connect(panel.set_image)
    panel.show()

    ros_timer = QTimer()
    def _spin():
        if rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0)
    ros_timer.timeout.connect(_spin)
    ros_timer.start(50)

    # Allow Ctrl+C to terminate the Qt app cleanly
    signal.signal(signal.SIGINT, lambda *_: app.quit())
    # Qt blocks Python signals — wake up the event loop periodically so SIGINT lands
    sigint_timer = QTimer()
    sigint_timer.start(200)
    sigint_timer.timeout.connect(lambda: None)

    exit_code = app.exec_()
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
