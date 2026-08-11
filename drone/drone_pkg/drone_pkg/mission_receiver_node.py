#!/usr/bin/env python3
"""
mission_receiver_node.py
--------------------------
Listens for mission files (JSON) pushed from the ground station over
ROSBridge, validates their structure, saves them to disk, and republishes
them on a ROS2 topic so the Navigator Package can pick them up and start
autonomous execution.

Mission JSON schema (produced by the Flask ground station's scan-pattern /
route-planning logic, see ground_station/server/mission_planner.py):

{
  "mission_id": "uuid",
  "created_at": "ISO8601",
  "home": {"lat": .., "lon": .., "alt": ..},
  "cruise_altitude_m": 30,
  "scan_altitude_m": 15,
  "scan_radius_m": 50,
  "waypoints": [
      {"seq": 0, "lat": .., "lon": .., "alt": .., "yaw": .., "action": "waypoint"},
      ...
      {"seq": N, "lat": .., "lon": .., "alt": .., "action": "rtl"}
  ]
}
"""

import glob
import json
import os
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

MISSION_STORAGE_DIR = os.environ.get('UAV_MISSION_DIR', '/home/pi/missions')
REQUIRED_TOP_LEVEL_KEYS = {'mission_id', 'waypoints'}
REQUIRED_WAYPOINT_KEYS = {'seq', 'lat', 'lon', 'alt'}

# BUGFIX: ground_station/server/ssh_uploader.py's SSH fallback path SCPs the
# mission file to MISSION_STORAGE_DIR and then calls a ROS2 service to
# trigger execution -- but that service never existed anywhere in the
# codebase (it called navigator_node/start_mission, and navigator_node has
# no service server at all, only a topic subscription). This service is the
# fix: it loads the most recently SCP'd file from disk and republishes it on
# 'drone/mission/active' exactly like the normal ROSBridge path does, so
# both delivery paths converge on the same code in navigator_node.


class MissionReceiverNode(Node):
    def __init__(self):
        super().__init__('mission_receiver_node')

        os.makedirs(MISSION_STORAGE_DIR, exist_ok=True)

        # Ground station -> Pi (raw mission JSON string, via rosbridge)
        self.mission_in_sub = self.create_subscription(
            String, 'ground_station/mission/upload', self._on_mission_received, 10
        )

        # Republished for navigator_pkg to consume once validated
        self.mission_out_pub = self.create_publisher(
            String, 'drone/mission/active', 10
        )
        self.mission_status_pub = self.create_publisher(
            String, 'drone/mission/status', 10
        )

        # SSH/SCP fallback trigger (see BUGFIX note above and
        # ssh_uploader.py's REMOTE_START_SERVICE_CMD)
        self.start_mission_srv = self.create_service(
            Trigger, 'mission_receiver_node/start_mission', self._on_start_mission_service
        )

        self.current_mission_path = None
        self.get_logger().info(
            f'mission_receiver_node started, storing missions under {MISSION_STORAGE_DIR}'
        )

    def _validate(self, mission: dict) -> (bool, str):
        missing = REQUIRED_TOP_LEVEL_KEYS - mission.keys()
        if missing:
            return False, f'Missing top-level keys: {missing}'
        if not isinstance(mission['waypoints'], list) or not mission['waypoints']:
            return False, 'waypoints must be a non-empty list'
        for i, wp in enumerate(mission['waypoints']):
            missing_wp = REQUIRED_WAYPOINT_KEYS - wp.keys()
            if missing_wp:
                return False, f'Waypoint {i} missing keys: {missing_wp}'
            if not (-90 <= wp['lat'] <= 90) or not (-180 <= wp['lon'] <= 180):
                return False, f'Waypoint {i} has out-of-range lat/lon'
        return True, 'ok'

    def _on_mission_received(self, msg: String):
        self._publish_status('receiving', 'Mission payload received, validating...')
        try:
            mission = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self._publish_status('error', f'Invalid JSON: {e}')
            return

        ok, reason = self._validate(mission)
        if not ok:
            self.get_logger().error(f'Mission rejected: {reason}')
            self._publish_status('rejected', reason)
            return

        mission_id = mission.get('mission_id', datetime.utcnow().strftime('%Y%m%dT%H%M%S'))
        path = os.path.join(MISSION_STORAGE_DIR, f'{mission_id}.json')
        with open(path, 'w') as f:
            json.dump(mission, f, indent=2)
        self.current_mission_path = path

        self.get_logger().info(
            f'Mission {mission_id} accepted ({len(mission["waypoints"])} waypoints), '
            f'saved to {path}'
        )
        self._publish_status('accepted', f'{len(mission["waypoints"])} waypoints loaded')

        out = String()
        out.data = json.dumps(mission)
        self.mission_out_pub.publish(out)

    def _on_start_mission_service(self, request, response):
        """Handles the SSH/SCP fallback trigger: loads whichever mission
        file was most recently written to MISSION_STORAGE_DIR (i.e. the one
        ssh_uploader.upload_mission() just SCP'd in) and republishes it on
        'drone/mission/active', same as the normal ROSBridge path."""
        files = sorted(
            glob.glob(os.path.join(MISSION_STORAGE_DIR, '*.json')),
            key=os.path.getmtime, reverse=True,
        )
        if not files:
            response.success = False
            response.message = f'No mission files found in {MISSION_STORAGE_DIR}'
            return response

        latest = files[0]
        try:
            with open(latest) as f:
                mission = json.load(f)
            ok, reason = self._validate(mission)
            if not ok:
                response.success = False
                response.message = f'Latest mission file {latest} invalid: {reason}'
                return response

            self.current_mission_path = latest
            out = String()
            out.data = json.dumps(mission)
            self.mission_out_pub.publish(out)
            self._publish_status('accepted_via_ssh', f'Started {os.path.basename(latest)} via SSH fallback')

            response.success = True
            response.message = f'Started mission {os.path.basename(latest)}'
        except (json.JSONDecodeError, OSError) as e:
            response.success = False
            response.message = f'Failed to load {latest}: {e}'
        return response

    def _publish_status(self, state: str, detail: str):
        payload = {
            'state': state,
            'detail': detail,
            'timestamp': self.get_clock().now().to_msg().sec,
        }
        msg = String()
        msg.data = json.dumps(payload)
        self.mission_status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MissionReceiverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
