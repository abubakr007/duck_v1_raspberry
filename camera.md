# Camera

## Detected hardware

A Raspberry Pi camera is connected to the CSI port of the Raspberry Pi 5.

From `media-ctl -d /dev/media0 -p`:

```
driver          rp1-cfe
model           rp1-cfe
- entity 16: imx708_wide
- entity 19: dw9807 10-000c
```

| Field    | Value                                      |
|----------|--------------------------------------------|
| Sensor   | Sony **IMX708** (wide variant)             |
| Module   | **Raspberry Pi Camera Module 3 Wide**      |
| Resolution | 12 MP (4608 x 2592)                      |
| Field of view | ~102° (wide)                          |
| Autofocus | Yes — Dongwoon **DW9807** VCM on I²C bus 10 (addr 0x0c) |
| CSI receiver | `rp1-cfe` (Pi 5 front-end) on `/dev/media0` |
| Video nodes | `/dev/video0` … `/dev/video7`            |

## Status of the userspace stack

This system runs a **hybrid libcamera stack**: stock Ubuntu Noble libcamera 0.2 alongside Raspberry Pi Foundation libcamera 0.5.

- **`libcamera 0.5.2`** and **`libcamera-dev 0.5.2`** are installed from the Raspberry Pi apt source. This is what actually drives the IMX708 on Pi 5 — it ships the `pisp` pipeline handler that Ubuntu's libcamera 0.2 lacks.
- **`libcamera-ipa 0.5.2`** ships the `ipa/rpi/pisp/imx708_wide.json` tuning file used at runtime.
- Ubuntu's stock **`libcamera0.2`** + **`libcamera-tools 0.2`** are still installed alongside (different SONAME — `libcamera.so.0.2` vs `libcamera.so.0.5`, peaceful coexistence). The Ubuntu `cam` CLI is linked to the old 0.2 library and **still reports zero cameras** — that is expected and not a failure. Use the C++/`libcamera-dev 0.5` path described below.
- Kernel + V4L2 detect the sensor (visible via `media-ctl` and `v4l2-ctl --list-devices`) — that part has always worked.

## Quick checks

```bash
# kernel-level detection
v4l2-ctl --list-devices
media-ctl -d /dev/media0 -p | grep -E "entity|driver|model"

# libcamera 0.5 path (Ubuntu's stale `cam` CLI is linked to 0.2 and will return empty —
# use the workspace helper instead, which links against libcamera-dev 0.5).
~/duck_ws/src/temp/cam_capture /tmp/check.ppm && echo OK
```

## Why this was broken on Ubuntu Noble + Pi 5

Two independent blockers, both Pi-5-specific:

1. **Ubuntu Noble ships libcamera 0.2.0**, which only contains the **`vc4`** pipeline handler (Pi 4 and earlier). Pi 5 needs the **`pisp`** pipeline handler, added in libcamera **0.3+**. Even though all the IMX708 IPA tuning JSONs were already present at `/usr/share/libcamera/ipa/rpi/vc4/imx708*.json`, libcamera 0.2 has no handler that can drive the `rp1-cfe` + `pispbe` graph, so `cam --list` returned empty.
2. **Direct V4L2 capture from `/dev/video0` fails** with `Failed to start media pipeline: -32` (EPIPE) at `STREAMON`. Root cause: the IMX708 sensor's embedded-data pad reports `unknown/28800x1`, but the `csi2` receiver hardcodes its embedded sink to `unknown/16384x1`. The link is `IMMUTABLE` and the csi2 driver rejects format changes with `EINVAL`. Pipeline link-validation fails before any frame is produced. Only libcamera (with the pisp pipeline handler) knows how to negotiate around this.

`rpicam-apps` from the Raspberry Pi apt source does **not** install cleanly on Ubuntu Noble:

- `librpicam-app1` depends on `libjpeg62-turbo` — Noble only ships `libjpeg-turbo8` (SONAME `.so.8`). No drop-in replacement.
- `rpicam-apps-encoder` depends on `libavcodec59` / `libavformat59` (ffmpeg 5) — Noble has ffmpeg 6 (`libavcodec60`).
- `python3-libcamera` from the same source is built for Python 3.11 ABI — Noble ships Python 3.12, so the `.so` will not load.

So we install only the **runtime + headers** from the PPA and use the libcamera C++ API directly.

## Install steps that work on Ubuntu Noble

```bash
# 1. Add the Raspberry Pi apt source
echo "deb http://archive.raspberrypi.com/debian bookworm main" \
    | sudo tee /etc/apt/sources.list.d/raspi.list
curl -sSL https://archive.raspberrypi.com/debian/raspberrypi.gpg.key \
    | sudo gpg --dearmor -o /etc/apt/trusted.gpg.d/raspberrypi.gpg
sudo apt update

# 2. Pin the PPA to high priority *only* for camera-related packages, so it
#    cannot accidentally pull in other Bookworm packages on a future upgrade.
sudo tee /etc/apt/preferences.d/raspberrypi-camera >/dev/null <<'EOF'
Package: libcamera* python3-libcamera python3-picamera2 libpisp* rpicam*
Pin: origin archive.raspberrypi.com
Pin-Priority: 990

Package: *
Pin: origin archive.raspberrypi.com
Pin-Priority: 100
EOF

# 3. Install only the runtime + dev headers (no rpicam-apps, no python bindings).
sudo apt install -y libcamera0.5 libcamera-ipa libpisp1 libcamera-dev
```

This bumps `libcamera-ipa` from 0.2 → 0.5 and pulls in `libpisp1`, `libpisp-common`, and a few `libboost-*1.74` deps. **Nothing gets removed** — Ubuntu's `libcamera0.2` and `gstreamer1.0-libcamera` stay in place.

Verify the install:

```bash
pkg-config --modversion libcamera          # → 0.5.2
ls /usr/share/libcamera/pipeline/rpi/      # → pisp  vc4   (pisp is the new bit)
ls /usr/share/libcamera/ipa/rpi/pisp/      # → should contain imx708_wide.json
```

## Capturing an image

A minimal libcamera C++ capture program lives at `temp/cam_capture.cpp` in this workspace. It opens the first camera, configures a 2304×1296 BGR888 still stream, runs 5 warm-up frames (so AE/AWB stabilises), and writes the 6th frame to a PPM file.

```bash
cd ~/duck_ws/src/temp
g++ -std=c++17 -O2 cam_capture.cpp $(pkg-config --cflags --libs libcamera) -o cam_capture
./cam_capture capture.ppm
ffmpeg -y -loglevel error -i capture.ppm -q:v 3 capture.jpg
```

The captured image is ~90° rotated relative to natural viewing because the IMX708 module's `camera_sensor_rotation` is 180° and the program does not apply an `Orientation` control. Add `controls::Rotation = 180` (or pass a `Transform` in the configuration) if you need it upright.

## ROS 2 integration options

Both off-the-shelf paths have caveats on this hybrid stack:

| Option | Package | Status on this system |
|---|---|---|
| libcamera-based | `ros-${ROS_DISTRO}-camera-ros` | Linked against the **Ubuntu libcamera 0.2** ABI (different SONAME from the new `libcamera.so.0.5` we installed). It will not see the IMX708 until rebuilt from source against `libcamera-dev 0.5.2`. |
| Plain V4L2 | `ros-${ROS_DISTRO}-v4l2-camera` | **Will not work** — it streams from `/dev/video0`, which hits the IMMUTABLE embedded-data link mismatch and gets `EPIPE` from the kernel. Bypassing libcamera is not an option on Pi 5 + IMX708. |

The realistic plan for `duck_vision` is to **build a small ROS 2 node from source** that links against `libcamera-dev 0.5.2` and uses the same C++ API as `temp/cam_capture.cpp`, then publishes `sensor_msgs/Image` + `sensor_msgs/CameraInfo`. The capture program in `temp/` is meant to be the seed of that node.

## Notes specific to Pi 5 / this workspace

- On Pi 5 the legacy `vcgencmd get_camera` no longer works (`Command not registered`) — that command was for the old Broadcom firmware path. Use `media-ctl` / `v4l2-ctl` / the `cam_capture` helper instead.
- The CSI bridge always exposes `/dev/video0`–`/dev/video7` and `/dev/media0` even with no sensor attached, so the presence of these device nodes alone is **not** proof a camera is connected — you need to see an actual sensor entity (`imx708_wide` here) in the media graph.
- The `pispbe` entries on `/dev/media1`–`/dev/media2` are the Pi 5 image-signal-processor back-end, and `/dev/media3` is the `rpivid` H.264/HEVC decoder — neither of these is a camera.
- **Do not run `apt remove libcamera0.2` or `apt autoremove` aggressively.** Ubuntu's stale 0.2 library is harmless, but several reverse-deps still pin it. Removing it would also remove `gstreamer1.0-libcamera` and `libcamera-tools` (the `cam` CLI), neither of which is in active use here but which would obscure the root cause if a future debugging session ran into them.
- **`rpicam-hello` / `rpicam-still` are not installed and cannot be installed** on this system without rebuilding `librpicam-app1` from source against `libjpeg-turbo8` and `libavcodec60`. See "Why this was broken" above for why.
- If a future Claude session sees `cam --list` returning nothing and assumes the camera stack is broken: it is not. The `cam` binary is the stale Ubuntu 0.2 build. Run `~/duck_ws/src/temp/cam_capture /tmp/x.ppm` or any program linked against `libcamera-dev` (0.5.2) to confirm enumeration.
