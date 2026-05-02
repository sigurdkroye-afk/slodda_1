#!/usr/bin/env python3
"""
Combined YOLO camera view + mission control buttons in a single window.
Replaces rqt_image_view and the standalone control_panel.
"""
import sys
import threading
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QFrame, QSizePolicy,
)
from PyQt5.QtCore import Qt, pyqtSignal, QObject
from PyQt5.QtGui import QImage, QPixmap, QFont

BUTTONS = [
    ('APPROVE',     '✓ Approve',     ['AT_BEAR']),
    ('RETURN_HOME', '⟵ Return Home', ['NAVIGATE', 'CANCELING', 'TRACK_BEAR', 'AT_BEAR']),
    ('ABORT',       '✕ Abort',       ['NAVIGATE', 'CANCELING', 'TRACK_BEAR']),
]

STATE_COLORS = {
    'IDLE':        '#888888',
    'NAVIGATE':    '#4a9fe8',
    'CANCELING':   '#4a9fe8',
    'TRACK_BEAR':  '#e8a020',
    'AT_BEAR':     '#40c040',
    'RETURN_HOME': '#c060c0',
}


class _Signals(QObject):
    image_ready  = pyqtSignal(QImage)
    status_ready = pyqtSignal(str)


class _Node(Node):
    def __init__(self, sig: _Signals):
        super().__init__('camera_control_panel')
        self._sig    = sig
        self._bridge = CvBridge()
        self.pub = self.create_publisher(String, '/mission/cmd', 10)
        self.create_subscription(Image,  '/yolo/image',     self._img_cb,    10)
        self.create_subscription(String, '/mission/status', self._status_cb, 10)

    def _img_cb(self, msg: Image):
        try:
            bgr  = self._bridge.imgmsg_to_cv2(msg, 'bgr8')
            rgb  = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
            h, w = rgb.shape[:2]
            data = rgb.tobytes()
            qi   = QImage(data, w, h, 3 * w, QImage.Format_RGB888).copy()
            self._sig.image_ready.emit(qi)
        except Exception as e:
            self.get_logger().warn(f'img_cb: {e}')

    def _status_cb(self, msg: String):
        self._sig.status_ready.emit(msg.data)

    def send(self, cmd: str):
        self.pub.publish(String(data=cmd))


class CameraControlPanel(QWidget):
    def __init__(self, node: _Node):
        super().__init__()
        self._node  = node
        self._state = 'IDLE'

        self.setWindowTitle('Bear Mission')
        self.setMinimumSize(660, 520)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(5)

        # ── Camera image ──────────────────────────────────────────────────────
        self._img_label = QLabel('Waiting for camera...')
        self._img_label.setAlignment(Qt.AlignCenter)
        self._img_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self._img_label.setStyleSheet('background: #111; color: #555;')
        layout.addWidget(self._img_label, stretch=1)

        # ── Separator ─────────────────────────────────────────────────────────
        sep = QFrame()
        sep.setFrameShape(QFrame.HLine)
        layout.addWidget(sep)

        # ── Status label ──────────────────────────────────────────────────────
        self._status = QLabel('---')
        self._status.setAlignment(Qt.AlignCenter)
        f = QFont('monospace', 10)
        f.setBold(True)
        self._status.setFont(f)
        self._status.setTextFormat(Qt.RichText)
        layout.addWidget(self._status)

        # ── Buttons ───────────────────────────────────────────────────────────
        row = QHBoxLayout()
        row.setSpacing(6)
        self._btns: dict[str, QPushButton] = {}
        for cmd, label, _ in BUTTONS:
            btn = QPushButton(label)
            btn.setMinimumHeight(40)
            btn.clicked.connect(lambda _, c=cmd: self._node.send(c))
            row.addWidget(btn)
            self._btns[cmd] = btn
        layout.addLayout(row)

        self._refresh_buttons()

    def set_image(self, qi: QImage):
        px     = QPixmap.fromImage(qi)
        scaled = px.scaled(
            self._img_label.width(), self._img_label.height(),
            Qt.KeepAspectRatio, Qt.SmoothTransformation,
        )
        self._img_label.setPixmap(scaled)

    def set_status(self, state: str):
        self._state = state
        color = STATE_COLORS.get(state, '#888888')
        self._status.setText(
            f'<span style="color:{color};font-size:11pt">{state}</span>')
        self._refresh_buttons()

    def _refresh_buttons(self):
        for cmd, _, active_states in BUTTONS:
            self._btns[cmd].setEnabled(self._state in active_states)


def main(args=None):
    rclpy.init(args=args)
    app  = QApplication(sys.argv)
    sig  = _Signals()
    node = _Node(sig)

    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    panel = CameraControlPanel(node)
    sig.image_ready.connect(panel.set_image)
    sig.status_ready.connect(panel.set_status)
    panel.show()

    exit_code = app.exec_()
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)
