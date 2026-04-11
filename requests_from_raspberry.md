# Requests from Raspberry Pi

These are ROS2-side changes needed on the Raspberry Pi to support the new Flutter app features.

---

## 1. System Manager Node (`duck_system_manager`)

**Purpose**: Expose shutdown and service restart as ROS2 services.

**Package**: Can be added to `duck_bringup` or as a standalone package.

### Node: `duck_system_manager`

**Services**:

| Service | Type | Action |
|---------|------|--------|
| `/duck/shutdown` | `std_srvs/srv/Trigger` | Runs `sudo shutdown -h now` |
| `/duck/restart_service` | `std_srvs/srv/Trigger` | Runs `sudo systemctl restart duck_robot` |

**Implementation** (Python):
```python
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import subprocess

class DuckSystemManager(Node):
    def __init__(self):
        super().__init__('duck_system_manager')
        self.create_service(Trigger, '/duck/shutdown', self.shutdown_cb)
        self.create_service(Trigger, '/duck/restart_service', self.restart_cb)
        self.get_logger().info('System manager ready')

    def shutdown_cb(self, request, response):
        self.get_logger().warn('Shutdown requested from app!')
        response.success = True
        response.message = 'Shutting down...'
        subprocess.Popen(['sudo', 'shutdown', '-h', 'now'])
        return response

    def restart_cb(self, request, response):
        self.get_logger().warn('Service restart requested from app!')
        response.success = True
        response.message = 'Restarting duck_robot service...'
        subprocess.Popen(['sudo', 'systemctl', 'restart', 'duck_robot'])
        return response

def main():
    rclpy.init()
    node = DuckSystemManager()
    rclpy.spin(node)
    rclpy.shutdown()
```

### Sudoers Configuration

Add to `/etc/sudoers.d/duck_robot`:
```
abubakr ALL=(ALL) NOPASSWD: /sbin/shutdown
abubakr ALL=(ALL) NOPASSWD: /bin/systemctl restart duck_robot
```
(Replace `abubakr` with the actual username on the RPi)

### Systemd Service

**Important**: This node must NOT be part of the `duck_robot` service (otherwise it dies when restarting `duck_robot`). Create a separate systemd unit:

**File**: `/etc/systemd/system/duck_system_manager.service`
```ini
[Unit]
Description=Duck System Manager (shutdown/restart services)
After=network.target

[Service]
Type=simple
User=abubakr
ExecStart=/bin/bash -c "source /opt/ros/humble/setup.bash && source /home/abubakr/duck_ws/install/setup.bash && ros2 run duck_bringup duck_system_manager"
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
```

Enable it:
```bash
sudo systemctl enable duck_system_manager
sudo systemctl start duck_system_manager
```

---

## 2. Map Manager Node (`duck_map_manager`)

**Purpose**: Manage map files (list, load, save, delete, set default) from the Flutter app.

**Package**: Can be added to `duck_nav_stack` or `duck_bringup`.

### Communication Protocol (Topic-based RPC)

Uses pub/sub instead of custom service types to avoid creating an interfaces package.

| Topic | Type | Direction |
|-------|------|-----------|
| `/duck/map_manager/command` | `std_msgs/String` | Flutter -> RPi |
| `/duck/map_manager/response` | `std_msgs/String` | RPi -> Flutter |

Messages are JSON-encoded strings with a request `id` for correlation.

### Commands

#### List Maps
```json
// Request
{"id": "abc123", "cmd": "list"}

// Response
{"id": "abc123", "success": true, "data": {"maps": ["small_house", "my_house", "office"]}}
```

#### Load Map
Loads a map into map_server (calls `/map_server/load_map` service internally).
```json
// Request
{"id": "abc123", "cmd": "load", "name": "small_house"}

// Response
{"id": "abc123", "success": true, "message": "Map loaded"}
```

#### Save Current Map
Saves the current map_server map to disk with the given name (calls map_saver or reads from `/map` topic).
```json
// Request
{"id": "abc123", "cmd": "save", "name": "new_map_name"}

// Response
{"id": "abc123", "success": true, "message": "Map saved"}
```

#### Save Edited Map
Saves modified occupancy grid data sent from the Flutter app.
```json
// Request
{
  "id": "abc123",
  "cmd": "save_edited",
  "name": "my_house_edited",
  "width": 200,
  "height": 200,
  "resolution": 0.05,
  "origin_x": -5.0,
  "origin_y": -5.0,
  "data": [0, 0, 100, -1, 0, ...]
}

// Response
{"id": "abc123", "success": true, "message": "Edited map saved and loaded"}
```

#### Delete Map
```json
// Request
{"id": "abc123", "cmd": "delete", "name": "old_map"}

// Response
{"id": "abc123", "success": true, "message": "Map deleted"}
```

#### Set Default Map
Sets which map loads automatically on next boot.
```json
// Request
{"id": "abc123", "cmd": "set_default", "name": "small_house"}

// Response
{"id": "abc123", "success": true, "message": "Default map set"}
```

### Implementation Notes

**Map storage directory**: `~/duck_ws/src/duck_localization/maps/` (or a configurable path)

**File operations**:
- `list`: Glob `*.yaml` files in maps directory, return filenames without extension
- `load`: Call `map_server`'s `/map_server/load_map` service with full path to the `.yaml` file
- `save`: Use `nav2_map_server`'s map_saver_cli or manually:
  1. Subscribe to `/map` topic once, get current occupancy grid
  2. Write `.pgm` file (P5 format, binary PGM)
  3. Write companion `.yaml` file with metadata
- `save_edited`: Receive occupancy grid data from Flutter, write `.pgm` + `.yaml`
- `delete`: Remove `.yaml` and `.pgm` files
- `set_default`: Write map name to `~/.duck_active_map` text file

**Default map on boot**: Modify `duck_app.launch.py` (or `navigation.launch.py`) to read `~/.duck_active_map` and use that map path. Fallback to `small_house` if file doesn't exist.

Example launch file change in `navigation.launch.py`:
```python
import os

def get_default_map():
    active_map_file = os.path.expanduser('~/.duck_active_map')
    if os.path.exists(active_map_file):
        with open(active_map_file) as f:
            map_name = f.read().strip()
        map_path = os.path.join(maps_dir, f'{map_name}.yaml')
        if os.path.exists(map_path):
            return map_path
    return os.path.join(maps_dir, 'small_house.yaml')  # fallback
```

### PGM File Writing Reference

When saving edited map data as `.pgm`:
```python
def save_pgm(filepath, width, height, data):
    """Save occupancy grid as PGM (P5 binary format).
    
    data: list of int (-1=unknown, 0=free, 100=occupied)
    PGM values: 254=free, 0=occupied, 205=unknown
    """
    with open(filepath, 'wb') as f:
        f.write(f'P5\n{width} {height}\n255\n'.encode())
        for y in range(height - 1, -1, -1):  # flip Y axis
            for x in range(width):
                val = data[y * width + x]
                if val == -1:
                    f.write(bytes([205]))  # unknown
                elif val == 0:
                    f.write(bytes([254]))  # free
                else:
                    f.write(bytes([0]))    # occupied
```

Companion `.yaml` file:
```yaml
image: {map_name}.pgm
mode: trinary
resolution: {resolution}
origin: [{origin_x}, {origin_y}, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
```

### Where to Add This Node

Add `duck_map_manager` to the `duck_app.launch.py` launch file so it starts with the rest of the stack:
```python
duck_map_manager = Node(
    package='duck_bringup',  # or duck_nav_stack
    executable='duck_map_manager',
    name='duck_map_manager',
    parameters=[{'maps_directory': maps_dir}],
)
```

### rosbridge Configuration

If edited maps are large (e.g., 400x400 = 160,000 cells), the JSON message may exceed rosbridge's default max message size. Add to rosbridge launch:
```python
ros2 run rosbridge_server rosbridge_websocket \
  --ros-args -p delay_between_messages:=0.0 -p max_message_size:=10000000
```

---

## Summary Checklist

- [ ] Create `duck_system_manager` Python node with `/duck/shutdown` and `/duck/restart_service` services
- [ ] Configure sudoers for passwordless shutdown and systemctl restart
- [ ] Create separate systemd unit for `duck_system_manager` (independent of `duck_robot`)
- [ ] Create `duck_map_manager` Python node with topic-based RPC for map CRUD
- [ ] Modify launch file to read default map from `~/.duck_active_map`
- [ ] Add `duck_map_manager` to `duck_app.launch.py`
- [ ] Optionally increase rosbridge `max_message_size` for large map transfers
