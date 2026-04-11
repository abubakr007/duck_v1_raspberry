# Raspberry Pi 5 - BLE Pairing Service Requirements

The Flutter app needs a BLE (Bluetooth Low Energy) GATT server running on the Raspberry Pi 5 so that new users can configure WiFi and get the robot's IP address without needing to know it in advance.

## When to Run

The BLE pairing service should **always run in the background** as a systemd service. It must be active so the Flutter app can discover and connect to the Pi via Bluetooth at any time (e.g., when the robot moves to a new WiFi network).

## BLE GATT Server Specification

### Advertised Name

The device should advertise as `DuckV1` (or `DuckV1-XXXX` where XXXX is the last 4 of the MAC address for uniqueness). The name must be non-empty so the Flutter app can display it in the scan results.

### Service UUID

```
12345678-1234-5678-1234-56789abcdef0
```

### Characteristics

| UUID | Name | Properties | Description |
|---|---|---|---|
| `12345678-1234-5678-1234-56789abcdef1` | WiFi SSID | Write | The Flutter app writes the WiFi network name (UTF-8 encoded) |
| `12345678-1234-5678-1234-56789abcdef2` | WiFi Password | Write | The Flutter app writes the WiFi password (UTF-8 encoded) |
| `12345678-1234-5678-1234-56789abcdef3` | Command | Write | The Flutter app writes a command string (see below) |
| `12345678-1234-5678-1234-56789abcdef4` | Status | Read, Notify | The Pi sends status updates back to the Flutter app |

### Command Values (written to Command characteristic)

| Command | Action |
|---|---|
| `CONNECT` | Use the previously written SSID and Password to connect to WiFi |

### Status Values (sent via Notify on Status characteristic)

| Status | Meaning |
|---|---|
| `CONNECTING` | Pi is attempting to connect to WiFi |
| `CONNECTED:<ip_address>` | Successfully connected. `<ip_address>` is the Pi's IP on the WiFi network (e.g., `CONNECTED:192.168.1.42`) |
| `FAILED:<reason>` | Connection failed. `<reason>` is a human-readable error (e.g., `FAILED:Wrong password`, `FAILED:Network not found`) |

## Flow

1. Flutter app scans for BLE devices and finds the Pi (by its advertised name)
2. Flutter app connects to the Pi's GATT server
3. Flutter app discovers services and finds service `12345678-1234-5678-1234-56789abcdef0`
4. Flutter app writes the WiFi SSID to characteristic `...def1`
5. Flutter app writes the WiFi password to characteristic `...def2`
6. Flutter app subscribes to notifications on status characteristic `...def4`
7. Flutter app writes `CONNECT` to command characteristic `...def3`
8. Pi receives the command, attempts to connect to the WiFi using `nmcli` or `networkctl`
9. Pi sends status notifications:
   - First: `CONNECTING`
   - Then either: `CONNECTED:192.168.x.x` or `FAILED:<reason>`
10. Flutter app reads the IP from the `CONNECTED:` message and uses it to connect to rosbridge at `ws://<ip>:9090`
11. Flutter app disconnects BLE (no longer needed)

## Implementation Notes

### Recommended Python Library

Use `bless` (Bluetooth Low Energy Server for Python) or `bluezero` for the GATT server implementation.

Install: `pip install bless` or `pip install bluezero`

### WiFi Connection

Use `nmcli` to connect to WiFi:

```bash
nmcli device wifi connect "<SSID>" password "<PASSWORD>"
```

After successful connection, get the IP address:

```bash
hostname -I | awk '{print $1}'
```

Or via Python:

```python
import subprocess
result = subprocess.run(['hostname', '-I'], capture_output=True, text=True)
ip = result.stdout.strip().split()[0]
```

### Getting the IP Address

After WiFi connection succeeds, wait a moment (1-2 seconds) for DHCP to assign an IP, then read it and send via the Status characteristic notification.

### systemd Service

Create a systemd service so the BLE server starts on boot:

```ini
[Unit]
Description=Duck V1 BLE Pairing Service
After=bluetooth.target
Wants=bluetooth.target

[Service]
Type=simple
ExecStart=/usr/bin/python3 /home/pi/duck_ble_pairing.py
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
```

### Bluetooth Setup on Pi

Make sure Bluetooth is enabled:

```bash
sudo systemctl enable bluetooth
sudo systemctl start bluetooth
```

The Pi 5 has built-in Bluetooth 5.0 which supports BLE.

### Security Note

The WiFi password is transmitted over BLE. While BLE has encryption at the link layer, for additional security you could add application-layer encryption in a future iteration. For now, the short range of BLE (~10m) provides reasonable security for a local robot pairing scenario.

---

# Camera Compressed Image Streaming (Phase 2)

The Flutter app subscribes to `/camera/image_raw/compressed` (`sensor_msgs/CompressedImage`) via rosbridge to display a live camera feed.

## Requirements

The `duck_vision` camera node already uses `image_transport::CameraPublisher`, which automatically publishes compressed image topics **if the image_transport plugins are installed**.

### Install image_transport plugins

```bash
sudo apt install ros-${ROS_DISTRO}-image-transport-plugins
```

This provides the `compressed` transport plugin which auto-publishes `/camera/image_raw/compressed` (JPEG) alongside the raw topic.

### Verify it works

After installing and restarting the camera node:

```bash
# Check the compressed topic exists
ros2 topic list | grep compressed

# Should show: /camera/image_raw/compressed

# Verify messages are flowing
ros2 topic hz /camera/image_raw/compressed
```

### Rosbridge note

Make sure rosbridge is launched **without** restricting message size, as compressed images can be 50-150KB encoded as base64:

```bash
ros2 run rosbridge_server rosbridge_websocket \
  --ros-args -p delay_between_messages:=0.0
```

No other changes needed on the RPi side - the existing camera node and rosbridge handle everything.
