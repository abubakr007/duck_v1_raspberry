import glob
import json
import os

import rclpy
from rclpy.node import Node
from nav2_msgs.srv import LoadMap
from std_msgs.msg import String


class DuckMapManager(Node):
    def __init__(self):
        super().__init__('duck_map_manager')

        self.declare_parameter(
            'maps_directory',
            os.path.expanduser('~/duck_ws/src/duck_localization/maps'),
        )
        self.maps_dir = self.get_parameter('maps_directory').value

        self.active_map_file = os.path.expanduser('~/.duck_active_map')

        # Topic-based RPC
        self.cmd_sub = self.create_subscription(
            String, '/duck/map_manager/command', self.on_command, 10,
        )
        self.resp_pub = self.create_publisher(
            String, '/duck/map_manager/response', 10,
        )

        # Client for map_server's LoadMap service
        self.load_map_client = self.create_client(
            LoadMap, '/map_server/load_map',
        )

        self.get_logger().info(
            f'Map manager ready — maps directory: {self.maps_dir}'
        )

    # ── command dispatcher ──────────────────────────────────────────

    def on_command(self, msg):
        try:
            req = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().error(f'Invalid JSON: {msg.data}')
            return

        req_id = req.get('id', '')
        cmd = req.get('cmd', '')

        handlers = {
            'list': self.handle_list,
            'load': self.handle_load,
            'save': self.handle_save,
            'save_edited': self.handle_save_edited,
            'delete': self.handle_delete,
            'set_default': self.handle_set_default,
        }

        handler = handlers.get(cmd)
        if handler is None:
            self.respond(req_id, False, message=f'Unknown command: {cmd}')
            return

        try:
            handler(req_id, req)
        except Exception as e:
            self.get_logger().error(f'Error handling "{cmd}": {e}')
            self.respond(req_id, False, message=str(e))

    # ── helpers ─────────────────────────────────────────────────────

    def respond(self, req_id, success, message=None, data=None):
        resp = {'id': req_id, 'success': success}
        if message is not None:
            resp['message'] = message
        if data is not None:
            resp['data'] = data
        out = String()
        out.data = json.dumps(resp)
        self.resp_pub.publish(out)

    # ── handlers ────────────────────────────────────────────────────

    def handle_list(self, req_id, _req):
        yaml_files = glob.glob(os.path.join(self.maps_dir, '*.yaml'))
        names = sorted(
            os.path.splitext(os.path.basename(f))[0] for f in yaml_files
        )
        self.respond(req_id, True, data={'maps': names})

    def handle_load(self, req_id, req):
        name = req.get('name', '')
        yaml_path = os.path.join(self.maps_dir, f'{name}.yaml')
        if not os.path.isfile(yaml_path):
            self.respond(req_id, False, message=f'Map not found: {name}')
            return

        if not self.load_map_client.wait_for_service(timeout_sec=5.0):
            self.respond(
                req_id, False, message='map_server load service unavailable',
            )
            return

        srv_req = LoadMap.Request()
        srv_req.map_url = yaml_path
        future = self.load_map_client.call_async(srv_req)
        future.add_done_callback(
            lambda f: self._load_map_done(f, req_id, name)
        )

    def _load_map_done(self, future, req_id, name):
        try:
            result = future.result()
            if result.result == LoadMap.Response.RESULT_SUCCESS:
                self.respond(req_id, True, message=f'Map "{name}" loaded')
            else:
                self.respond(
                    req_id, False,
                    message=f'map_server rejected load (code {result.result})',
                )
        except Exception as e:
            self.respond(req_id, False, message=str(e))

    def handle_save(self, req_id, req):
        name = req.get('name', '')
        if not name:
            self.respond(req_id, False, message='Missing "name"')
            return

        # Subscribe to /map once, grab the current occupancy grid, save it
        from nav_msgs.msg import OccupancyGrid

        def on_map(grid_msg):
            self.destroy_subscription(sub)
            try:
                self._write_map(name, grid_msg)
                self.respond(req_id, True, message=f'Map "{name}" saved')
            except Exception as e:
                self.respond(req_id, False, message=str(e))

        sub = self.create_subscription(OccupancyGrid, '/map', on_map, 1)

    def handle_save_edited(self, req_id, req):
        name = req.get('name', '')
        width = req.get('width')
        height = req.get('height')
        resolution = req.get('resolution')
        origin_x = req.get('origin_x')
        origin_y = req.get('origin_y')
        data = req.get('data')

        if not all([name, width, height, resolution, data]):
            self.respond(req_id, False, message='Missing required fields')
            return

        if origin_x is None:
            origin_x = 0.0
        if origin_y is None:
            origin_y = 0.0

        pgm_path = os.path.join(self.maps_dir, f'{name}.pgm')
        yaml_path = os.path.join(self.maps_dir, f'{name}.yaml')

        self._save_pgm(pgm_path, width, height, data)
        self._save_yaml(yaml_path, name, resolution, origin_x, origin_y)

        # Also load it into map_server
        if self.load_map_client.wait_for_service(timeout_sec=5.0):
            srv_req = LoadMap.Request()
            srv_req.map_url = yaml_path
            future = self.load_map_client.call_async(srv_req)
            future.add_done_callback(
                lambda f: self.respond(
                    req_id, True, message='Edited map saved and loaded',
                )
            )
        else:
            self.respond(
                req_id, True,
                message='Edited map saved (map_server not available to reload)',
            )

    def handle_delete(self, req_id, req):
        name = req.get('name', '')
        yaml_path = os.path.join(self.maps_dir, f'{name}.yaml')
        pgm_path = os.path.join(self.maps_dir, f'{name}.pgm')

        if not os.path.isfile(yaml_path):
            self.respond(req_id, False, message=f'Map not found: {name}')
            return

        os.remove(yaml_path)
        if os.path.isfile(pgm_path):
            os.remove(pgm_path)

        self.respond(req_id, True, message=f'Map "{name}" deleted')

    def handle_set_default(self, req_id, req):
        name = req.get('name', '')
        yaml_path = os.path.join(self.maps_dir, f'{name}.yaml')
        if not os.path.isfile(yaml_path):
            self.respond(req_id, False, message=f'Map not found: {name}')
            return

        with open(self.active_map_file, 'w') as f:
            f.write(name)

        self.respond(req_id, True, message=f'Default map set to "{name}"')

    # ── file I/O ────────────────────────────────────────────────────

    def _write_map(self, name, grid_msg):
        """Write an OccupancyGrid message to .pgm + .yaml."""
        info = grid_msg.info
        pgm_path = os.path.join(self.maps_dir, f'{name}.pgm')
        yaml_path = os.path.join(self.maps_dir, f'{name}.yaml')

        self._save_pgm(
            pgm_path, info.width, info.height, list(grid_msg.data),
        )
        self._save_yaml(
            yaml_path, name, info.resolution,
            info.origin.position.x, info.origin.position.y,
        )

    @staticmethod
    def _save_pgm(filepath, width, height, data):
        """Save occupancy grid as PGM (P5 binary).

        data values: -1=unknown, 0=free, 100=occupied
        PGM values:  205=unknown, 254=free, 0=occupied
        """
        with open(filepath, 'wb') as f:
            f.write(f'P5\n{width} {height}\n255\n'.encode())
            for y in range(height - 1, -1, -1):
                for x in range(width):
                    val = data[y * width + x]
                    if val == -1:
                        f.write(bytes([205]))
                    elif val == 0:
                        f.write(bytes([254]))
                    else:
                        f.write(bytes([0]))

    @staticmethod
    def _save_yaml(filepath, name, resolution, origin_x, origin_y):
        with open(filepath, 'w') as f:
            f.write(
                f'image: {name}.pgm\n'
                f'mode: trinary\n'
                f'resolution: {resolution}\n'
                f'origin: [{origin_x}, {origin_y}, 0.0]\n'
                f'negate: 0\n'
                f'occupied_thresh: 0.65\n'
                f'free_thresh: 0.196\n'
            )


def main(args=None):
    rclpy.init(args=args)
    node = DuckMapManager()
    rclpy.spin(node)
    rclpy.shutdown()
