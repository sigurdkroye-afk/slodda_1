#!/usr/bin/env python3
"""
python3 scripts/arm_uart_test.py          -> send 0 og q, vis svar
python3 scripts/arm_uart_test.py listen   -> lytt 10 sek (reset ESP32 naa!)
"""
import serial, time, sys

PORT = '/dev/ttyS0'
mode = sys.argv[1] if len(sys.argv) > 1 else 'send'

s = serial.Serial(PORT, 115200, timeout=1.0)
print(f'Port {PORT} aapen')

if mode == 'listen':
    print('Lytter 10 sek — reset ESP32 NAA og se om ARM_READY dukker opp...')
    deadline = time.time() + 10
    got = []
    while time.time() < deadline:
        line = s.readline()
        if line:
            decoded = line.decode('utf-8', errors='replace').strip()
            print('MOTTATT:', repr(decoded))
            got.append(decoded)
    if not got:
        print('Ingenting mottatt — ESP32 TX -> Pi RX er brutt (eller ESP32 ikke oppe)')
    else:
        print(f'Mottok {len(got)} linje(r) fra ESP32. TX->Pi fungerer.')
else:
    print('Sender 0 (hjem) — skal svare DONE:0 via Serial2...')
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
