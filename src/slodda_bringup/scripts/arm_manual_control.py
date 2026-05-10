#!/usr/bin/env python3
"""
Manuelt kalibreringsskript for TPU-armen via /arm/raw_command.
Krev at arm_controller_node kjører.
Kjør: python3 arm_manual_control.py
"""
import sys
import tty
import termios
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

HELP = """
=== ARM MANUELL KONTROLL ===
  t/g       -> Servo 1 steg inn/ut (1 sek)
  y/h       -> Servo 2 steg inn/ut
  u/j       -> Servo 3 steg inn/ut
  a / A     -> Servo 1 hold inn / stopp hold
  s / S     -> Servo 2 hold inn / stopp hold
  d / D     -> Servo 3 hold inn / stopp hold
  w / W     -> Servo 1 hold ut  / stopp hold
  e / E     -> Servo 2 hold ut  / stopp hold
  r / R     -> Servo 3 hold ut  / stopp hold
  0/1/2/3   -> Gå til posisjon 0/1/2/3
  z/x/c/v   -> Lagre posisjon 0/1/2/3
  q         -> Nødstopp
  ESC/Ctrl+C -> Avslutt
============================
"""

_HOLD_START = set('asdwer')
_HOLD_STOP  = set('ASDWER')
_HOLD_TO_STOP = {'a': 'A', 's': 'S', 'd': 'D', 'w': 'W', 'e': 'E', 'r': 'R'}
_VALID = set('tghyujasdwer012345zxcvbnqASDWER')


class _ArmManual(Node):
    def __init__(self):
        super().__init__('arm_manual_control')
        self._pub = self.create_publisher(String, '/arm/raw_command', 10)
        self.create_subscription(String, '/arm/status', self._status_cb, 10)
        self._held: set[str] = set()
        self.create_timer(0.05, self._hold_tick)  # 20 Hz

    def _status_cb(self, msg: String):
        sys.stdout.write(f'\r[ESP32] {msg.data}\n> ')
        sys.stdout.flush()

    def _hold_tick(self):
        for k in list(self._held):
            self._pub_char(k)

    def _pub_char(self, ch: str):
        m = String()
        m.data = ch
        self._pub.publish(m)

    def handle_key(self, k: str):
        if k not in _VALID:
            return
        if k in _HOLD_START:
            self._held.add(k)
        elif k in _HOLD_STOP:
            self._held.discard(k.lower())
            self._pub_char(k)
        else:
            self._pub_char(k)


def main(args=None):
    rclpy.init(args=args)
    node = _ArmManual()

    print(HELP)
    sys.stdout.write('> ')
    sys.stdout.flush()

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    spin_t = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_t.start()

    try:
        tty.setraw(fd)
        while True:
            ch = sys.stdin.read(1)
            if not ch or ch in ('\x1b', '\x03'):  # ESC or Ctrl+C
                break
            node.handle_key(ch)
            if ch == 'q':
                break
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        node._held.clear()
        node.destroy_node()
        rclpy.shutdown()
        print('\nAvsluttet.')


if __name__ == '__main__':
    main()
