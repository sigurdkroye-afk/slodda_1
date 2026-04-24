import numpy as np
import cv2
from ultralytics import YOLO

# 1. Oppsett
model = YOLO('/home/slodda1/yolov8n.pt')
w, h, stride = 2592, 1944, 3264

# 2. Ta bilde (Raw pGAA format)
import os
os.system("v4l2-ctl --device=/dev/video0 --set-fmt-video=width=2592,height=1944,pixelformat=pGAA --stream-mmap --stream-count=1 --stream-to=/tmp/raw.bin")

# 3. Unpack 10-bit
raw = np.fromfile('/tmp/raw.bin', dtype=np.uint8)
raw_rows = raw.reshape((h, stride))
row_data = raw_rows[:, :w*10//8].reshape(-1, 5)
raw_16 = np.zeros(w * h, dtype=np.uint16)
raw_16[0::4] = ((row_data[:,0].astype(np.uint16) << 2) | (row_data[:,4].astype(np.uint16) & 0x03))
raw_16[1::4] = ((row_data[:,1].astype(np.uint16) << 2) | ((row_data[:,4].astype(np.uint16) >> 2) & 0x03))
raw_16[2::4] = ((row_data[:,2].astype(np.uint16) << 2) | ((row_data[:,4].astype(np.uint16) >> 4) & 0x03))
raw_16[3::4] = ((row_data[:,3].astype(np.uint16) << 2) | ((row_data[:,4].astype(np.uint16) >> 6) & 0x03))

bayer = raw_16.reshape((h, w)).astype(np.uint8) # Forenklet for test
bgr = cv2.cvtColor(bayer, cv2.COLOR_BayerGB2BGR)

# 4. YOLO deteksjon
results = model(bgr, classes=[77], verbose=True) # 77 = teddy bear
cv2.imwrite('/home/slodda1/yolo_result.jpg', bgr)
print(f"Ferdig! Fant {len(results[0].boxes)} teddybjørner.")
