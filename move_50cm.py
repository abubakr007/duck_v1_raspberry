#!/usr/bin/env python3
"""Closed-loop serial control: move the duck robot a target distance using encoder feedback."""

import serial
import time
import re

PORT = "/dev/ttyACM0"
BAUD = 115200
WHEEL_RADIUS = 0.0335  # meters
WHEEL_SEP = 0.17       # meters
# Encoder over-reads by ~4.4%: reports 1.004m when actual is 0.96m
# Correction: target_encoder = desired / (actual/encoder) = desired / (0.96/1.004)
DIST_CORRECTION = 1.004 / 0.96
DISTANCE = 1.00 * DIST_CORRECTION  # encoder target for 1.00m actual
OMEGA = 2.0            # rad/s (base wheel angular velocity)
DRIFT_CORRECTION = 0.014  # rad/s — reduce left wheel to correct rightward drift

# Feedback regex: "rp05.23,lp04.87,"
FEEDBACK_RE = re.compile(r'r([pn])([\d.]+),l([pn])([\d.]+),')

omega_right = OMEGA
omega_left = OMEGA - DRIFT_CORRECTION
cmd = f"rp{omega_right:05.2f},lp{omega_left:05.2f},,"
stop = "rp00.00,lp00.00,,"

ser = serial.Serial(PORT, BAUD, timeout=0.05)
time.sleep(2)  # wait for Arduino reset
ser.reset_input_buffer()  # clear any stale data

print(f"Target: {DISTANCE}m | Right: {omega_right:.3f} rad/s, Left: {omega_left:.3f} rad/s")
print(f"Command: {cmd}")
print("-" * 60)

dist_right = 0.0
dist_left = 0.0
last_time = time.time()

try:
    while True:
        # Send command
        ser.write(cmd.encode())

        # Read all available feedback lines
        while ser.in_waiting:
            line = ser.readline().decode(errors='ignore').strip()
            m = FEEDBACK_RE.search(line)
            if m:
                now = time.time()
                dt = now - last_time
                last_time = now

                r_sign = 1.0 if m.group(1) == 'p' else -1.0
                r_vel = float(m.group(2)) * r_sign  # rad/s
                l_sign = 1.0 if m.group(3) == 'p' else -1.0
                l_vel = float(m.group(4)) * l_sign  # rad/s

                dist_right += r_vel * WHEEL_RADIUS * dt
                dist_left += l_vel * WHEEL_RADIUS * dt
                avg_dist = (dist_right + dist_left) / 2.0
                lateral = (dist_left - dist_right) / WHEEL_SEP * (avg_dist / 2.0) if avg_dist > 0 else 0

                print(f"\rDist: {avg_dist:.3f}m | R: {r_vel:.2f} L: {l_vel:.2f} rad/s | "
                      f"Drift: {lateral*100:.1f}cm", end="", flush=True)

                if avg_dist >= DISTANCE:
                    raise StopIteration

        time.sleep(0.05)  # 20 Hz send rate

except (KeyboardInterrupt, StopIteration):
    pass

# Stop
print(f"\nStopping... Final distance: {(dist_right + dist_left) / 2.0:.3f}m")
for _ in range(20):
    ser.write(stop.encode())
    time.sleep(0.05)

ser.close()
print("Done.")
