#!/usr/bin/env python3
"""Standalone control panel — publishes /mission/cmd, subscribes /mission/status."""
import sys
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

try:
    from PyQt5.QtWidgets import (
        QApplication, QWidget, QVBoxLayout,
        QPushButton, QLabel, QFrame,
    )
    from PyQt5.QtCore import Qt, pyqtSignal, QObject
    from PyQt5.QtGui import QFont
except ImportError:
    print('[control_panel] PyQt5 not found — install python3-pyqt5')
    sys.exit(1)


# Which buttons are enabled in each state
_BUTTONS = {
    'START':       ['IDLE'],
    'APPROVE':     ['AT_BEAR'],
    'RETURN_HOME': ['NAVIGATE', 'TRACK_BEAR', 'AT_BEAR', 'CANCELING'],
    'ABORT':       ['NAVIGATE', 'TRACK_BEAR', 'CANCELING'],
}

_STATE_COLORS = {
    'IDLE':        '#555555',
    'NAVIGATE':    '#1a6bb5',
    'CANCELING':   '#1a6bb5',
    'TRACK_BEAR':  '#c47a00',
    'AT_BEAR':     '#2d8a2d',
    'RETURN_HOME': "#862c86",
}


class _Signals(QObject):
    status_changed = pyqtSignal(str)


class _Node(Node):
    def __init__(self, signals):
        super().__init__('mission_control_panel')
        self._signals = signals
        self.pub = self.create_publisher(String, '/mission/cmd', 10)
        self.create_subscription(String, '/mission/status', self._cb, 10)

    def _cb(self, msg: String):
        self._signals.status_changed.emit(msg.data)

    def send(self, cmd: str):
        self.pub.publish(String(data=cmd))


class ControlPanel(QWidget):
    def __init__(self, node: _Node):
        super().__init__()
        self._node    = node
        self._state   = 'IDLE'

        self.setWindowTitle('Bear Mission Control')
        self.setMinimumWidth(300)

        layout = QVBoxLayout(self)
        layout.setSpacing(8)
        layout.setContentsMargins(16, 16, 16, 16)

        title = QLabel('Bear Mission Control')
        title.setAlignment(Qt.AlignCenter)
        title.setFont(QFont('sans-serif', 13, QFont.Bold))
        layout.addWidget(title)

        sep = QFrame()
        sep.setFrameShape(QFrame.HLine)
        layout.addWidget(sep)

        self._status_label = QLabel('Status: IDLE')
        self._status_label.setAlignment(Qt.AlignCenter)
        self._status_label.setFont(QFont('monospace', 11))
        layout.addWidget(self._status_label)

        layout.addSpacing(8)

        self._btns = {}
        defs = [
            ('START',       'Start Mission'),
            ('APPROVE',     'Approve at Bear'),
            ('RETURN_HOME', 'Return Home'),
            ('ABORT',       'Abort'),
        ]
        for cmd, label in defs:
            btn = QPushButton(label)
            btn.setMinimumHeight(44)
            btn.clicked.connect(lambda _, c=cmd: self._node.send(c))
            layout.addWidget(btn)
            self._btns[cmd] = btn

        self._refresh()

    def update_state(self, state: str):
        self._state = state
        color = _STATE_COLORS.get(state, '#555555')
        self._status_label.setText(f'Status: {state}')
        self._status_label.setStyleSheet(f'color: {color}; font-weight: bold;')
        self._refresh()

    def _refresh(self):
        for cmd, btn in self._btns.items():
            enabled = self._state in _BUTTONS.get(cmd, [])
            btn.setEnabled(enabled)
            btn.setStyleSheet('' if enabled else 'color: grey;')


def main(args=None):
    rclpy.init(args=args)
    app = QApplication(sys.argv)

    signals = _Signals()
    node    = _Node(signals)

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    panel = ControlPanel(node)
    signals.status_changed.connect(panel.update_state)
    panel.show()

    exit_code = app.exec_()

    node.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)
