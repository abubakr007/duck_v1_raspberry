# Duck Robot — Custom Startup Services

Everything below runs automatically on boot. No crontab or `/etc/rc.local` entries exist.

---

## 1. `duck_robot.service` — ROS2 Full Stack

**File:** `/etc/systemd/system/duck_robot.service`
**Script:** `/home/abubakr/duck_ws/start_duck.sh`
**Runs as:** `abubakr`
**Restart policy:** on-failure (5s delay)

### What it does

1. Sources `/opt/ros/jazzy/setup.bash` and the workspace overlay
2. Waits 5 seconds for hardware/network readiness
3. Launches `duck_app.launch.py`, which starts:
   - `real_robot.launch.py` (with `use_camera:=true`) — hardware interface, lidar, controllers, IMU, camera
   - `navigation.launch.py` (with map `my_house.yaml`) — map server, AMCL, Nav2 stack
   - `image_republisher` — bridges raw camera images to compressed JPEG (only when camera is enabled)
   - `rosbridge_websocket` — WebSocket server on port 9090 for Flutter app

### Environment variables

| Variable | Value |
|---|---|
| `RMW_IMPLEMENTATION` | `rmw_cyclonedds_cpp` |
| `ROS_DOMAIN_ID` | `7` |
| `ROS_LOCALHOST_ONLY` | `0` |
| `ROS_AUTOMATIC_DISCOVERY_RANGE` | `SUBNET` |

### Commands

```bash
sudo systemctl start duck_robot.service
sudo systemctl stop duck_robot.service
sudo systemctl restart duck_robot.service
sudo systemctl status duck_robot.service
journalctl -u duck_robot.service -f    # live logs
```

---

## 2. `duck-ble-pairing.service` — BLE Pairing for Flutter App

**File:** `/etc/systemd/system/duck-ble-pairing.service`
**Script:** `/home/abubakr/duck_ble_pairing.py`
**Runs as:** `abubakr`
**Restart policy:** always (5s delay)
**Starts after:** `bluetooth.target`, `dbus.service`

### What it does

Runs a BLE GATT server (using `bless` library) that:
- Advertises as `DuckV1-19FA` over Bluetooth Low Energy
- Accepts WiFi credentials (SSID + password) from the Flutter app
- Connects to WiFi via `nmcli` (or returns current IP if already on that network)
- Sends the Pi's IP address back to the Flutter app via BLE notification

### Service UUID

`12345678-1234-5678-1234-56789abcdef0`

### Commands

```bash
sudo systemctl start duck-ble-pairing.service
sudo systemctl stop duck-ble-pairing.service
sudo systemctl restart duck-ble-pairing.service
sudo systemctl status duck-ble-pairing.service
journalctl -u duck-ble-pairing.service -f    # live logs
```

---

## 3. `wifi-reset-button.service` — GPIO Button to Restart SSH

**File:** `/etc/systemd/system/wifi-reset-button.service`
**Script:** `/home/abubakr/duck_ws/src/temp/wifi_reset_button.py`
**Runs as:** `root`
**Restart policy:** always (2s delay)
**GPIO pin factory:** `lgpio`

### What it does

Listens for a physical button press on **GPIO 17** (pull-up, with debounce). When pressed:
1. Restarts the SSH service (`sudo systemctl restart ssh`)
2. Blinks an LED on **GPIO 27** for 2 seconds as visual feedback

### Commands

```bash
sudo systemctl start wifi-reset-button.service
sudo systemctl stop wifi-reset-button.service
sudo systemctl restart wifi-reset-button.service
sudo systemctl status wifi-reset-button.service
```

---

## 4. Polkit Rules — NetworkManager Access

### JavaScript rule (polkit 124+)

**File:** `/etc/polkit-1/rules.d/50-allow-nmcli.rules`

Allows user `abubakr` to manage NetworkManager (WiFi connect/disconnect) without password authentication. Required by the BLE pairing service to run `nmcli` as a non-root user.

### Legacy rule (unused, kept for reference)

**File:** `/etc/polkit-1/localauthority/50-local.d/allow-nmcli.pkla`

Not effective on polkit 124+ (uses JavaScript rules instead).

---

## 5. Shell Environment (`~/.bashrc`)

ROS-related exports that apply to interactive shells (not to systemd services):

```bash
export ROS_DOMAIN_ID=7
export ROS_LOCALHOST_ONLY=0
export ROS_IP=192.168.1.194
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

> **Note:** The systemd `duck_robot.service` has its own environment block — changes to `.bashrc` do not affect it. Keep both in sync.
