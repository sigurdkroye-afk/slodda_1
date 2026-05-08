#!/usr/bin/env python3
"""Quick encoder diagnostic — run while motors are spinning."""
import lgpio, time

LEFT_ENC_A_PIN = 17

h = lgpio.gpiochip_open(0)

print(f"lgpio.BOTH_EDGES={lgpio.BOTH_EDGES} RISING={lgpio.RISING_EDGE} FALLING={lgpio.FALLING_EDGE}")

print("\n=== TEST 1: polling GPIO17 (200 samples, 5ms apart) ===")
lgpio.gpio_claim_input(h, LEFT_ENC_A_PIN, lgpio.SET_PULL_UP)
vals = []
for _ in range(200):
    vals.append(lgpio.gpio_read(h, LEFT_ENC_A_PIN))
    time.sleep(0.005)
transitions = sum(1 for i in range(1, len(vals)) if vals[i] != vals[i-1])
print(f"values: min={min(vals)} max={max(vals)} transitions={transitions}")
if transitions == 0:
    print("  → signal NOT changing — wiring or hardware issue")
else:
    print(f"  → signal IS toggling ({transitions} edges seen)")

lgpio.gpiochip_close(h)
h = lgpio.gpiochip_open(0)

print("\n=== TEST 2: callback (no lFlags) ===")
count = [0]

def on_tick(chip, gpio, level, tick):
    count[0] += 1

try:
    lgpio.gpio_claim_alert(h, LEFT_ENC_A_PIN, lgpio.BOTH_EDGES)
    print("  gpio_claim_alert(BOTH_EDGES) OK")
except Exception as e:
    print(f"  BOTH_EDGES failed: {e}, trying RISING|FALLING=3")
    lgpio.gpio_claim_alert(h, LEFT_ENC_A_PIN, 3)

cb = lgpio.callback(h, LEFT_ENC_A_PIN, lgpio.BOTH_EDGES, on_tick)
print("Waiting 5s — spin the left wheel by hand now...")
time.sleep(5)
print(f"  → callback ticks received: {count[0]}")
cb.cancel()
lgpio.gpiochip_close(h)
