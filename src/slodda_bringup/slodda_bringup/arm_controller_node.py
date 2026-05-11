#!/usr/bin/env python3
import re
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

try:
    import serial
    _SERIAL_OK = True
except ImportError:
    _SERIAL_OK = False


class ArmControllerNode(Node):
    def __init__(self):
        super().__init__('arm_controller')

        self._ser = None
        self._status_pub = self.create_publisher(String, '/arm/status', 10)

        self._done = {i: threading.Event() for i in range(6)}
        self._grabbed = threading.Event()
        self._stopped = threading.Event()

        if not _SERIAL_OK:
            self.get_logger().error('pyserial ikke installert — pip install pyserial')
        else:
            try:
                self._ser = serial.Serial('/dev/ttyS0', 115200, timeout=0.1)
                self.get_logger().info('Serial /dev/ttyAMA0 åpnet.')
            except Exception as e:
                self.get_logger().error(f'Kan ikke åpne /dev/ttyAMA0: {e}')

        self._running = True
        threading.Thread(target=self._reader_loop, daemon=True).start()

        if self._ser is not None:
            self.get_logger().info('Hjem arm til posisjon 0...')
            self._ser.write(b'0')
            if self._done[0].wait(timeout=10.0):
                self.get_logger().info('Arm i posisjon 0.')
            else:
                self.get_logger().warn('Hjem-timeout — fortsetter uansett.')

        self.create_service(Trigger, '/arm/open',   self._svc_open)
        self.create_service(Trigger, '/arm/drive',  self._svc_drive)
        self.create_service(Trigger, '/arm/search', self._svc_search)
        self.create_service(Trigger, '/arm/grab',   self._svc_grab)
        self.create_service(Trigger, '/arm/stop',   self._svc_stop)

        self.create_subscription(String, '/arm/raw_command', self._raw_cb, 10)

        self.get_logger().info('ArmControllerNode klar.')

    def _reader_loop(self):
        while self._running:
            if self._ser is None:
                import time
                time.sleep(0.1)
                continue
            try:
                line = self._ser.readline().decode('utf-8', errors='ignore').strip()
                if not line:
                    continue
                self.get_logger().info(f'ESP32: {line}')
                msg = String()
                msg.data = line
                self._status_pub.publish(msg)

                # Robust parsing: tolererer UART-garbling (f.eks. "DONE2" for "DONE:2")
                m = re.search(r'DONE[^0-9]*(\d)', line)
                if m:
                    pos = int(m.group(1))
                    canonical = f'DONE:{pos}'
                    msg.data = canonical
                    if pos == 2:
                        self._grabbed.clear()
                    if pos in self._done:
                        self._done[pos].set()
                elif 'GRABBED' in line:
                    msg.data = 'GRABBED'
                    self._grabbed.set()
                elif 'STOPPED' in line:
                    msg.data = 'STOPPED'
                    self._stopped.set()
            except Exception:
                pass

    def _send_wait(self, cmd: bytes, event: threading.Event, timeout: float, ok_msg: str):
        if self._ser is None:
            return False, 'Serial port ikke tilgjengelig'
        try:
            event.clear()
            self._ser.write(cmd)
            if event.wait(timeout=timeout):
                return True, ok_msg
            return False, 'TIMEOUT'
        except Exception as e:
            return False, str(e)

    def _svc_open(self, _req, resp):
        ok, msg = self._send_wait(b'0', self._done[0], 10.0, 'DONE:0')
        resp.success, resp.message = ok, msg
        return resp

    def _svc_drive(self, _req, resp):
        ok, msg = self._send_wait(b'1', self._done[1], 10.0, 'DONE:1')
        resp.success, resp.message = ok, msg
        return resp

    def _svc_search(self, _req, resp):
        ok, msg = self._send_wait(b'2', self._done[2], 10.0, 'DONE:2')
        resp.success, resp.message = ok, msg
        return resp

    def _svc_grab(self, _req, resp):
        ok, msg = self._send_wait(b'3', self._done[3], 10.0, 'DONE:3')
        resp.success, resp.message = ok, msg
        return resp

    def _svc_stop(self, _req, resp):
        ok, msg = self._send_wait(b'q', self._stopped, 3.0, 'STOPPED')
        resp.success, resp.message = ok, msg
        return resp

    def _raw_cb(self, msg: String):
        if self._ser is None or not msg.data:
            return
        try:
            self._ser.write(msg.data[0].encode())
            self.get_logger().info(f'RAW: {msg.data[0]}')
        except Exception as e:
            self.get_logger().warn(f'RAW send feilet: {e}')

    def destroy_node(self):
        if self._ser is not None:
            try:
                self._done[0].clear()
                self._ser.write(b'0')
                self._done[0].wait(timeout=5.0)
                self._ser.close()
            except Exception:
                pass
        self._running = False
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArmControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
