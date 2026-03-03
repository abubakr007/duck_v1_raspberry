# Arduino PID Controller Fix

## Issues Found

### 1. PID Sample Time Mismatch (Critical)
The PID_v1 library defaults to a 200ms sample time, but the control loop runs every 100ms.
This meant the PID only computed every other tick — halving responsiveness and causing slow convergence.

**Fix:** Added `SetSampleTime(interval)` in `setup()` to match the 100ms loop.

### 2. String Objects Used in ISRs (Critical)
`right_wheel_sign` and `left_wheel_sign` were `String` objects modified inside interrupt callbacks.
`String` uses dynamic heap allocation — calling it in an ISR risks heap corruption and random crashes.

**Fix:** Changed from `String` to `volatile char` (`'p'`/`'n'`).

### 3. Race Condition on Wheel Signs (Critical)
Encoder counters were read atomically inside a `noInterrupts()` block, but wheel direction signs
were read later when building the feedback string. The sign could change between reads if the
wheel reversed direction mid-cycle.

**Fix:** Signs are now captured inside the same `noInterrupts()` block as the counters, stored
in `last_r_sign`/`last_l_sign` for use in the feedback string.

### 4. Motor Driver Naming
Code comments and pin defines referenced "L298N" but the actual hardware is a TB6612FNG.

**Fix:** Renamed all `L298N_*` defines to `TB6612_*`.

## PID Gain Re-tuning

After fixing the sample time from 200ms to 100ms, the original gains were too weak.
The integral term accumulated at half the rate it was tuned for.

| Parameter | Old (200ms sample) | New (100ms sample) |
|-----------|-------------------|--------------------|
| Kp_r      | 11.5              | 20.0               |
| Ki_r      | 7.5               | 60.0               |
| Kd_r      | 0.1               | 0.2                |
| Kp_l      | 12.8              | 22.0               |
| Ki_l      | 8.3               | 65.0               |
| Kd_l      | 0.1               | 0.2                |

### Tuning iterations at 2.0 rad/s setpoint

1. Original gains (Ki ~7.5): reached ~1.05 rad/s after 6s — far too slow
2. Doubled Ki (~15): reached ~1.5 rad/s after 6s — better but still slow
3. Ki ~30, Kp ~15: reached ~1.8 rad/s after 5s — approaching target
4. Ki ~60, Kp ~20 (final): reaches ~1.96-1.99 rad/s in ~3.5s, steady state within 2% error

## Test Results (Final Gains)

### Forward (2.0 rad/s, 5 seconds)
- Smooth ramp, no overshoot or oscillation
- Reaches ~1.96 rad/s by 3.8s, steady at 1.96-1.99
- Both wheels track within 0.03 rad/s of each other

### Reverse (2.0 rad/s, 5 seconds)
- Converges to 1.99-2.03 rad/s within 0.5s (PID integral pre-warmed)
- Rock steady for the full duration
- Direction signs correct: `rn`/`ln` throughout

### Stop behavior
- Decelerates to 0 within ~0.3s, no ringing

## Tools Setup

Installed `arduino-cli` v1.4.1 to `~/bin` for compiling and uploading:
```bash
export PATH="$HOME/bin:$PATH"
arduino-cli compile --fqbn arduino:avr:mega /tmp/last_arduino/last_arduino.ino
arduino-cli upload -p /dev/ttyACM0 --fqbn arduino:avr:mega /tmp/last_arduino/last_arduino.ino
```
