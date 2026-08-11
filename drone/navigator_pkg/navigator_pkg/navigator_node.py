#!/usr/bin/env python3
"""
navigator_node.py
--------------------
Executes the autonomous flight mission end-to-end (FYP report, Ch. 4.2.3.2):

  1. Waits for a validated mission on 'drone/mission/active' (published by
     drone_pkg/mission_receiver_node).
  2. Converts the mission's waypoints into MAVROS WaypointPush requests and
     uploads them to the Pixhawk over MAVLink (UART @ 57600 baud, per
     Ch. 5.1.2).
  3. Arms the vehicle, switches to AUTO mode, and monitors mission
     progress via /mavros/mission/reached and /mavros/state.
  4. Hands temporary control to Guided mode when the obstacle_avoidance
     package reports an obstacle (see obstacle_avoidance_node.py), then
     resumes AUTO once clear.
  5. Publishes mission status back to the ground station.

This node is the ROS2 counterpart of the "Navigator" lane in the FYP's
full mission flowchart (Figure 38).
"""

import json
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String, Bool
from sensor_msgs.msg import NavSatFix

from mavros_msgs.msg import State, WaypointReached
from mavros_msgs.srv import CommandBool, SetMode, WaypointPush, WaypointClear
from mavros_msgs.msg import Waypoint

# NOTE (bugfix): the previous version of this node spawned a plain
# threading.Thread for mission execution and called
# rclpy.spin_until_future_complete(self, future) from inside it, while the
# main thread was already running rclpy.spin(node). Two threads cannot spin
# the same node's executor at once -- this either raises or deadlocks the
# instant a mission is uploaded. The fix: all service clients/subscriptions
# now share a ReentrantCallbackGroup, the node runs under a
# MultiThreadedExecutor (see main()), and async calls are awaited with a
# threading.Event + add_done_callback instead of nested spinning.


FRAME_GLOBAL_REL_ALT = 3          # MAV_FRAME_GLOBAL_RELATIVE_ALT
CMD_NAV_WAYPOINT = 16             # MAV_CMD_NAV_WAYPOINT
CMD_NAV_RETURN_TO_LAUNCH = 20     # MAV_CMD_NAV_RETURN_TO_LAUNCH
CMD_NAV_LAND = 21                 # MAV_CMD_NAV_LAND


class NavigatorNode(Node):
    def __init__(self):
        super().__init__('navigator_node')

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                          history=HistoryPolicy.KEEP_LAST)

        # All callbacks share one reentrant group so a service response can
        # be processed by a worker thread while the mission-execution
        # callback (running on another worker thread) is still waiting on
        # it -- required for the MultiThreadedExecutor in main().
        cb_group = ReentrantCallbackGroup()

        # --- State ---
        self.current_state = State()
        self.current_gps = None
        self.mission_waypoints = []
        self.mission_active = False
        self.avoidance_active = False
        self.last_mode_before_avoidance = 'AUTO'

        # --- Subscriptions ---
        self.create_subscription(String, 'drone/mission/active', self._on_new_mission, 10,
                                  callback_group=cb_group)
        self.create_subscription(State, 'mavros/state', self._on_state, qos,
                                  callback_group=cb_group)
        self.create_subscription(NavSatFix, 'mavros/global_position/global', self._on_gps, qos,
                                  callback_group=cb_group)
        self.create_subscription(WaypointReached, 'mavros/mission/reached', self._on_wp_reached, 10,
                                  callback_group=cb_group)
        self.create_subscription(Bool, 'drone/obstacle_avoidance/active', self._on_avoidance_flag, 10,
                                  callback_group=cb_group)

        # --- Publishers ---
        self.status_pub = self.create_publisher(String, 'drone/mission/status', 10)

        # --- Service clients (MAVROS) ---
        self.arming_client = self.create_client(CommandBool, 'mavros/cmd/arming',
                                                  callback_group=cb_group)
        self.set_mode_client = self.create_client(SetMode, 'mavros/set_mode',
                                                    callback_group=cb_group)
        self.wp_push_client = self.create_client(WaypointPush, 'mavros/mission/push',
                                                   callback_group=cb_group)
        self.wp_clear_client = self.create_client(WaypointClear, 'mavros/mission/clear',
                                                    callback_group=cb_group)

        self.get_logger().info('navigator_node started, waiting for mission...')

    # ------------------------------------------------------------------ #
    # Callbacks
    # ------------------------------------------------------------------ #
    def _on_state(self, msg: State):
        self.current_state = msg

    def _on_gps(self, msg: NavSatFix):
        self.current_gps = msg

    def _on_wp_reached(self, msg: WaypointReached):
        self._publish_status('waypoint_reached', f'Reached waypoint {msg.wp_seq}')
        if self.mission_waypoints and msg.wp_seq == len(self.mission_waypoints) - 1:
            self._publish_status('mission_complete', 'Final waypoint reached')
            self.mission_active = False

    def _on_avoidance_flag(self, msg: Bool):
        """Reacts to the obstacle_avoidance package taking/releasing control."""
        if msg.data and not self.avoidance_active:
            self.avoidance_active = True
            self.last_mode_before_avoidance = self.current_state.mode or 'AUTO'
            self._set_mode('GUIDED')
            self._publish_status('avoidance', 'Obstacle detected, ceding control to avoidance logic')
        elif not msg.data and self.avoidance_active:
            self.avoidance_active = False
            self._set_mode(self.last_mode_before_avoidance)
            self._publish_status('avoidance_cleared', 'Obstacle cleared, resuming mission')

    def _on_new_mission(self, msg: String):
        try:
            mission = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().error('Failed to parse mission JSON')
            return

        self.get_logger().info(
            f"New mission received: {len(mission['waypoints'])} waypoints"
        )
        threading.Thread(target=self._execute_mission, args=(mission,), daemon=True).start()

    # ------------------------------------------------------------------ #
    # Mission execution pipeline
    # ------------------------------------------------------------------ #
    def _execute_mission(self, mission: dict):
        self._publish_status('uploading', 'Clearing previous mission and uploading new one')

        self._wait_for_service(self.wp_clear_client)
        self._call_sync(self.wp_clear_client, WaypointClear.Request(), timeout=10.0)

        waypoints = self._build_mavros_waypoints(mission['waypoints'])
        self.mission_waypoints = waypoints

        self._wait_for_service(self.wp_push_client)
        req = WaypointPush.Request()
        req.start_index = 0
        req.waypoints = waypoints
        result = self._call_sync(self.wp_push_client, req, timeout=15.0)

        if result is None or not result.success:
            self._publish_status('error', 'Waypoint upload failed')
            return

        self._publish_status('uploaded', f'{result.wp_transfered} waypoints uploaded')

        if not self._arm(True):
            self._publish_status('error', 'Arming failed (pre-arm checks not satisfied)')
            return

        if not self._set_mode('AUTO'):
            self._publish_status('error', 'Failed to switch to AUTO mode')
            return

        self.mission_active = True
        self._publish_status('launched', 'AUTO mode engaged, mission underway')

    def _build_mavros_waypoints(self, wp_list):
        waypoints = []
        for i, wp in enumerate(wp_list):
            mavros_wp = Waypoint()
            mavros_wp.frame = FRAME_GLOBAL_REL_ALT
            action = wp.get('action', 'waypoint')
            if action == 'rtl':
                mavros_wp.command = CMD_NAV_RETURN_TO_LAUNCH
            elif action == 'land':
                mavros_wp.command = CMD_NAV_LAND
            else:
                mavros_wp.command = CMD_NAV_WAYPOINT
            mavros_wp.is_current = (i == 0)
            mavros_wp.autocontinue = True
            mavros_wp.param1 = 0.0   # hold time (s)
            mavros_wp.param2 = 2.0   # acceptance radius (m)
            mavros_wp.param3 = 0.0   # pass-through radius
            mavros_wp.param4 = wp.get('yaw', float('nan'))
            mavros_wp.x_lat = wp['lat']
            mavros_wp.y_long = wp['lon']
            mavros_wp.z_alt = wp['alt']
            waypoints.append(mavros_wp)
        return waypoints

    # ------------------------------------------------------------------ #
    # MAVROS helpers
    # ------------------------------------------------------------------ #
    def _wait_for_service(self, client, timeout=10.0):
        if not client.wait_for_service(timeout_sec=timeout):
            self.get_logger().warn(f'Service {client.srv_name} not available')

    def _call_sync(self, client, request, timeout=10.0):
        """Blocks the CALLING thread (a mission-execution worker thread,
        never the executor thread) until the async service call completes,
        using a plain threading.Event fired by add_done_callback.

        This is safe under a MultiThreadedExecutor: the executor's other
        worker threads keep processing callbacks (including this service's
        response) while this thread waits, unlike
        rclpy.spin_until_future_complete(), which requires the calling
        thread itself to be the one driving the executor.
        """
        done_event = threading.Event()
        future = client.call_async(request)
        future.add_done_callback(lambda _f: done_event.set())

        if not done_event.wait(timeout=timeout):
            self.get_logger().error(f'Service call to {client.srv_name} timed out')
            return None
        if future.exception() is not None:
            self.get_logger().error(f'Service call to {client.srv_name} raised: {future.exception()}')
            return None
        return future.result()

    def _arm(self, value: bool) -> bool:
        self._wait_for_service(self.arming_client)
        req = CommandBool.Request()
        req.value = value
        result = self._call_sync(self.arming_client, req, timeout=10.0)
        return result is not None and result.success

    def _set_mode(self, mode: str) -> bool:
        self._wait_for_service(self.set_mode_client)
        req = SetMode.Request()
        req.custom_mode = mode
        result = self._call_sync(self.set_mode_client, req, timeout=10.0)
        ok = result is not None and result.mode_sent
        if ok:
            self.get_logger().info(f'Mode switched to {mode}')
        return ok

    def _publish_status(self, state: str, detail: str):
        payload = {'state': state, 'detail': detail,
                   'timestamp': self.get_clock().now().to_msg().sec}
        msg = String()
        msg.data = json.dumps(payload)
        self.status_pub.publish(msg)
        self.get_logger().info(f'[{state}] {detail}')


def main(args=None):
    rclpy.init(args=args)
    node = NavigatorNode()
    # MultiThreadedExecutor (not rclpy.spin) is required here: mission
    # execution runs on its own worker thread and blocks on service
    # responses (see _call_sync), so the executor needs spare threads free
    # to deliver those responses concurrently instead of a single spinner
    # thread that would otherwise deadlock against itself.
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
