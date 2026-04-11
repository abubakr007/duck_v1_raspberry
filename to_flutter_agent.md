# To Flutter Agent — ROS2-Side Implementation Notes

These nodes are now implemented and running on the Raspberry Pi. Below are the exact interfaces the Flutter app should integrate with via rosbridge.

---

## 1. System Manager

Two ROS2 services exposed over rosbridge:

| Service | Type | What it does |
|---------|------|--------------|
| `/duck/shutdown` | `std_srvs/srv/Trigger` | Shuts down the Raspberry Pi |
| `/duck/restart_service` | `std_srvs/srv/Trigger` | Restarts the `duck_robot` systemd service |

**Response fields** (both services):
- `success` (bool): always `true` if the command was accepted
- `message` (string): human-readable status

**Note**: The system manager runs as a **separate systemd unit** (`duck_system_manager.service`), so it survives `duck_robot` restarts. After calling `/duck/restart_service`, expect rosbridge and all other nodes to drop and reconnect — the system manager itself stays up.

---

## 2. Map Manager

Uses **topic-based JSON RPC** (not ROS services) to avoid custom message types.

| Topic | Type | Direction |
|-------|------|-----------|
| `/duck/map_manager/command` | `std_msgs/String` | Flutter → RPi |
| `/duck/map_manager/response` | `std_msgs/String` | RPi → Flutter |

All messages are JSON strings. Every request must include a unique `id` field; the response echoes it back for correlation.

### Commands

#### `list` — Get available maps
```json
// Request
{"id": "abc123", "cmd": "list"}

// Response
{"id": "abc123", "success": true, "data": {"maps": ["my_house", "small_house"]}}
```

#### `load` — Load a map into the navigation stack
```json
// Request
{"id": "abc123", "cmd": "load", "name": "small_house"}

// Response (success)
{"id": "abc123", "success": true, "message": "Map \"small_house\" loaded"}

// Response (failure — map_server not ready or file missing)
{"id": "abc123", "success": false, "message": "map_server load service unavailable"}
```

#### `save` — Save the current live map to disk
Captures the current `/map` topic (occupancy grid) and writes it as PGM + YAML.
```json
// Request
{"id": "abc123", "cmd": "save", "name": "new_map_name"}

// Response
{"id": "abc123", "success": true, "message": "Map \"new_map_name\" saved"}
```

#### `save_edited` — Save a modified map from the Flutter editor
Sends the full occupancy grid data from the app. The RPi writes it to disk and loads it into map_server.
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
  "data": [0, 0, 100, -1, 0]
}

// Response
{"id": "abc123", "success": true, "message": "Edited map saved and loaded"}
```

**Data values**: `-1` = unknown, `0` = free, `100` = occupied (standard ROS OccupancyGrid format).

**Size warning**: For a 400x400 map, `data` is 160,000 integers. The rosbridge `max_message_size` may need to be increased on the RPi side if this fails (currently not configured — default is ~10MB which should be fine for most maps, but watch for errors).

#### `delete` — Delete a map from disk
```json
// Request
{"id": "abc123", "cmd": "delete", "name": "old_map"}

// Response
{"id": "abc123", "success": true, "message": "Map \"old_map\" deleted"}
```

#### `set_default` — Set which map loads on next boot
Writes the map name to `~/.duck_active_map`. On next boot, `navigation.launch.py` reads this file and uses that map. Fallback is `my_house.yaml`.
```json
// Request
{"id": "abc123", "cmd": "set_default", "name": "small_house"}

// Response
{"id": "abc123", "success": true, "message": "Default map set to \"small_house\""}
```

### Error responses

All commands return this format on failure:
```json
{"id": "abc123", "success": false, "message": "Human-readable error description"}
```

---

## 3. Correlation ID pattern

The Flutter app should generate a unique `id` per request (UUID or timestamp-based) and match it against incoming responses on `/duck/map_manager/response`. Multiple commands can be in-flight simultaneously — the `id` is the only way to correlate them.

---

## 4. Rosbridge connection notes

- The map manager starts with the main `duck_app.launch.py` stack, so it's available as soon as rosbridge connects.
- The system manager runs independently — it's always available even if the main stack is down.
- After calling `/duck/restart_service`, the Flutter app should expect the WebSocket to drop and implement reconnection logic.
