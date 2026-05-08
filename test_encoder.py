#!/usr/bin/env python3
"""Quick encoder diagnostic — run while motors are spinning."""
import lgpio, time

LEFT_ENC_A_PIN = 17

h = lgpio.gpiochip_open(0)

print("=== TEST 1: polling GPIO17 (200 samples, 5ms apart) ===")
lgpio.gpio_claim_alert(h, LEFT_ENC_A_PIN, lgpio.BOTH_EDGES, lgpio.SET_PULL_UP)
vals = []
for _ in range(200):
    vals.append(lgpio.gpio_read(h, LEFT_ENC_A_PIN))
    time.sleep(0.005)
transitions = sum(1 for i in range(1, len(vals)) if vals[i] != vals[i-1])
print(f"values: min={min(vals)} max={max(vals)} transitions={transitions}")
if transitions == 0:
    print("  → signal NOT changing — wiring/hardware issue")
else:
    print(f"  → signal IS toggling ({transitions} edges seen)")

lgpio.gpiochip_close(h)
h = lgpio.gpiochip_open(0)

print("\n=== TEST 2: callback via gpio_claim_alert ===")
count = [0]

def on_tick(chip, gpio, level, tick):
    count[0] += 1

lgpio.gpio_claim_alert(h, LEFT_ENC_A_PIN, lgpio.BOTH_EDGES, lgpio.SET_PULL_UP)
cb = lgpio.callback(h, LEFT_ENC_A_PIN, lgpio.BOTH_EDGES, on_tick)
print("Waiting 5s — spin the motor now...")
time.sleep(5)
print(f"  → callback ticks received: {count[0]}")
cb.cancel()
lgpio.gpiochip_close(h)
