#!/usr/bin/env python3
import serial, time

PORT = '/dev/ttyS0'
s = serial.Serial(PORT, 115200, timeout=0.5)
print(f'Port {PORT} aapen')
print('Sender t (servo1 steg, 1s hardkodet)...')
s.write(b't')
time.sleep(3)
r = s.read_all()
print('Svar:', repr(r))
s.close()
