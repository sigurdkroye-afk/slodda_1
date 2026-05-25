#!/usr/bin/env python3
import serial, time

PORT = '/dev/ttyS0'
s = serial.Serial(PORT, 115200, timeout=0.5)
print(f'Port {PORT} aapen')
print('Sender 0 (hjem-posisjon) — skal svare DONE:0 via Serial2...')
s.write(b'0')
time.sleep(5)
r = s.read_all()
print('Svar etter 0:', repr(r))

time.sleep(1)
print('Sender q (nodstopp) — skal svare STOPPED via Serial2...')
s.write(b'q')
time.sleep(2)
r = s.read_all()
print('Svar etter q:', repr(r))
s.close()
