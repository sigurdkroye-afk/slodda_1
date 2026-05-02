import threading, time
from http.server import BaseHTTPRequestHandler, HTTPServer
from socketserver import ThreadingMixIn
from picamera2 import Picamera2
from libcamera import controls
from ultralytics import YOLO
import cv2

BEAR_CLASSES = [21, 77]  # 'bear' og 'teddy bear'

model = YOLO('/home/slodda1/yolov8n.pt')

picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"format": "RGB888", "size": (640, 480)}))
picam2.start()

# La AWB stabilisere seg, bruk tungsten-modus for innendørs kunstig lys
time.sleep(3)
picam2.set_controls({"AwbEnable": True, "AwbMode": controls.AwbModeEnum.Tungsten})
time.sleep(2)

latest_frame = None
frame_lock = threading.Lock()
frame_event = threading.Event()


def capture_loop():
    global latest_frame
    while True:
        frame = picam2.capture_array()

        results = model(frame, classes=BEAR_CLASSES, verbose=False, conf=0.25)
        for box in results[0].boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            conf = float(box.conf[0])
            cls_name = model.names[int(box.cls[0])]
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 200, 255), 2)
            cv2.putText(frame, f'{cls_name} {conf:.0%}', (x1, y1 - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 200, 255), 2)

        if not results[0].boxes:
            cv2.putText(frame, 'Ingen bamse...', (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (80, 80, 255), 2)

        _, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 75])
        with frame_lock:
            latest_frame = jpeg.tobytes()
        frame_event.set()
        frame_event.clear()


class Handler(BaseHTTPRequestHandler):
    def log_message(self, f, *a):
        pass

    def do_GET(self):
        if self.path == '/stream':
            self.send_response(200)
            self.send_header('Cache-Control', 'no-cache, private')
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
            self.end_headers()
            try:
                while True:
                    frame_event.wait(timeout=2)
                    with frame_lock:
                        frame = latest_frame
                    if frame:
                        self.wfile.write(
                            b'--frame\r\n'
                            b'Content-Type: image/jpeg\r\n'
                            b'Content-Length: ' + str(len(frame)).encode() + b'\r\n'
                            b'\r\n' + frame + b'\r\n'
                        )
                        self.wfile.flush()
            except Exception:
                pass
        else:
            self.send_response(404)
            self.end_headers()


class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    allow_reuse_address = True
    daemon_threads = True


threading.Thread(target=capture_loop, daemon=True).start()
print('Stream: http://172.20.10.2:5000/stream')
ThreadedHTTPServer(('', 5000), Handler).serve_forever()
