#!/usr/bin/env python3
"""
obstacle_avoidance_node.py
-----------------------------
Consumes the 360-degree scan from the YDLiDAR X4 Pro (published as
sensor_msgs/LaserScan by the vendor ROS2 driver) and implements the
avoidance logic described in the FYP report (Ch. 4.2.3.3):

  1. Build a real-time occupancy view of the surroundings from the scan.
  2. If any point falls inside SAFETY_RADIUS_M, raise the avoidance flag
     (drone/obstacle_avoidance/active) so navigator_node cedes control
     and switches the vehicle to GUIDED mode.
  3. Compute an evasive velocity command (the clearest heading, opposite
     the closest obstacle sector) and publish it as a MAVROS
     PositionTarget / velocity setpoint.
  4. Once the path is clear for CLEAR_HOLD_SEC seconds, drop the flag so
     navigator_node resumes AUTO and normal waypoint tracking.

Validated in RViz simulation and open-field flight tests to +/-2 cm
tolerance against ground-truth measurements (Ch. 5.3.5).
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from geometry_msgs.msg import TwistStamped

SAFETY_RADIUS_M = 2.0        # trigger avoidance inside this range
CRITICAL_RADIUS_M = 0.8      # emergency stop / hard evasive maneuver
CLEAR_HOLD_SEC = 1.5         # path must be clear this long before resuming AUTO
EVASIVE_SPEED_MPS = 0.6
NUM_SECTORS = 36             # 10-degree sectors for coarse occupancy


class ObstacleAvoidanceNode(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance_node')

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                          history=HistoryPolicy.KEEP_LAST)

        self.create_subscription(LaserScan, 'scan', self._on_scan, qos)

        self.active_pub = self.create_publisher(Bool, 'drone/obstacle_avoidance/active', 10)
        self.cmd_vel_pub = self.create_publisher(
            TwistStamped, 'mavros/setpoint_velocity/cmd_vel', 10
        )

        self.obstacle_present = False
        self.last_clear_time = time.time()
        self.avoidance_active = False

        self.get_logger().info(
            f'obstacle_avoidance_node started (safety radius={SAFETY_RADIUS_M} m)'
        )

    def _on_scan(self, msg: LaserScan):
        sectors = self._bin_into_sectors(msg)
        min_range, min_angle_idx = self._closest_obstacle(sectors)

        if min_range is None:
            self._maybe_clear()
            return

        if min_range <= SAFETY_RADIUS_M:
            self._trigger_avoidance(sectors, min_range, min_angle_idx)
        else:
            self._maybe_clear()

    def _bin_into_sectors(self, msg: LaserScan):
        """Reduces the raw scan into NUM_SECTORS angular bins, taking the
        minimum valid range in each bin (a coarse but robust occupancy view
        that tolerates noisy single-point returns)."""
        sectors = [float('inf')] * NUM_SECTORS
        angle = msg.angle_min
        for r in msg.ranges:
            if msg.range_min < r < msg.range_max:
                sector_idx = int(((angle % (2 * math.pi)) / (2 * math.pi)) * NUM_SECTORS)
                sector_idx = min(sector_idx, NUM_SECTORS - 1)
                if r < sectors[sector_idx]:
                    sectors[sector_idx] = r
            angle += msg.angle_increment
        return sectors

    def _closest_obstacle(self, sectors):
        min_range = min(sectors)
        if math.isinf(min_range):
            return None, None
        return min_range, sectors.index(min_range)

    def _clearest_sector(self, sectors):
        max_range = max(sectors)
        return sectors.index(max_range)

    def _trigger_avoidance(self, sectors, min_range, obstacle_sector_idx):
        if not self.avoidance_active:
            self.avoidance_active = True
            flag = Bool(); flag.data = True
            self.active_pub.publish(flag)
            self.get_logger().warn(
                f'Obstacle detected at {min_range:.2f} m (sector {obstacle_sector_idx}) '
                f'-> switching to evasive control'
            )

        clearest_idx = self._clearest_sector(sectors)
        heading_rad = (clearest_idx / NUM_SECTORS) * 2 * math.pi

        speed = EVASIVE_SPEED_MPS
        if min_range <= CRITICAL_RADIUS_M:
            speed = 0.0  # hard stop, do not translate further into the obstacle
            self.get_logger().error(
                f'CRITICAL: obstacle at {min_range:.2f} m -- holding position'
            )

        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'
        cmd.twist.linear.x = speed * math.cos(heading_rad)
        cmd.twist.linear.y = speed * math.sin(heading_rad)
        cmd.twist.linear.z = 0.0
        self.cmd_vel_pub.publish(cmd)

        self.last_clear_time = time.time()  # reset the "clear" timer

    def _maybe_clear(self):
        if self.avoidance_active and (time.time() - self.last_clear_time) >= CLEAR_HOLD_SEC:
            self.avoidance_active = False
            flag = Bool(); flag.data = False
            self.active_pub.publish(flag)
            self.get_logger().info('Path clear -- releasing control back to mission navigator')


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
