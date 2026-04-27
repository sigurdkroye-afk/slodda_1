from picamera2 import Picamera2
import cv2, time, threading
from http.server import BaseHTTPRequestHandler, HTTPServer
from socketserver import ThreadingMixIn

class ThreadedHTTPServer(ThreadingMixIn, HTTPServer): pass

picam2 = Picamera2()
config = picam2.create_preview_configuration(main={"format": "RGB888", "size": (640, 480)})
picam2.configure(config)
picam2.start()
time.sleep(2)

tracker = None
tracking = False
latest_frame = None
latest_bgr = None
lock = threading.Lock()

def loop():
    global tracker, tracking, latest_frame, latest_bgr
    while True:
        raw = picam2.capture_array()
        frame = cv2.cvtColor(raw, cv2.COLOR_RGB2BGR)
        if tracking and tracker is not None:
            success, bbox = tracker.update(frame)
            if success:
                x, y, w, h = [int(v) for v in bbox]
                cv2.rectangle(frame, (x,y), (x+w,y+h), (0,255,0), 2)
                cv2.putText(frame, 'Tracking', (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
            else:
                tracking = False
                cv2.putText(frame, 'Lost', (20,40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,0,255), 2)
        else:
            cv2.putText(frame, 'No target', (20,40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,165,255), 2)
        ret, jpeg = cv2.imencode('.jpg', frame)
        with lock:
            latest_frame = jpeg.tobytes()
            latest_bgr = frame.copy()
        time.sleep(0.05)

class Handler(BaseHTTPRequestHandler):
    def log_message(self, f, *a): pass
    def do_GET(self):
        global tracker, tracking
        if self.path == '/init':
            with lock:
                frame = latest_bgr.copy() if latest_bgr is not None else None
            if frame is None:
                self.send_response(503); self.end_headers()
                self.wfile.write(b'Not ready'); return
            h, w = frame.shape[:2]
            bbox = (w//4, h//4, w//2, h//2)
            tracker = cv2.legacy.TrackerCSRT_create()
            tracker.init(frame, bbox)
            tracking = True
            self.send_response(200); self.end_headers()
            self.wfile.write(b'Tracker initialized')
        elif self.path == '/stream':
            self.send_response(200)
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
            self.end_headers()
            try:
                while True:
                    with lock:
                        f = latest_frame
                    if f:
                        part = (
                            b'--frame\r\n'
                            b'Content-Type: image/jpeg\r\n'
                            b'Content-Length: ' + str(len(f)).encode() + b'\r\n'
                            b'\r\n' + f + b'\r\n'
                        )
                        self.wfile.write(part)
                        self.wfile.flush()
                    time.sleep(0.05)
            except (BrokenPipeError, ConnectionResetError):
                pass

threading.Thread(target=loop, daemon=True).start()
print('Stream: http://172.20.10.2:5000/stream')
print('Init:   http://172.20.10.2:5000/init')
ThreadedHTTPServer(('0.0.0.0', 5000), Handler).serve_forever()
