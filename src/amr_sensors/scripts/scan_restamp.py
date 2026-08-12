#!/usr/bin/env python3
# Copyright 2026 Ganbat Selenge
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# scan_restamp.py — re-stamp /scan_raw at a TF-consistent time and
# republish as /scan.
#
# Why: under headless software rendering (llvmpipe) the gpu_lidar's render
# thread runs seconds behind the physics clock, so raw scan messages carry
# stale timestamps and consumers (AMCL, SLAM, costmaps) drop them against
# the TF cache.
#
# Anchor choice: the LATEST map->odom /tf stamp, not /clock. The map->odom
# edge is produced by AMCL, which lags /clock by its processing latency;
# a scan stamped at /clock asks consumers for transforms at a time AMCL
# has not reached yet, so the costmap's message filter drops it
# ("timestamp ... earlier than all the data") or, while the robot turns,
# transforms it with a stale yaw — walls smear around the robot and DWB
# reports "no legal trajectories" / "Start occupied". Stamping at the
# latest map->odom time makes every scan resolvable against the TF chain.
# Falls back to /clock before the first map->odom transform exists.
import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage


class ScanRestamper(Node):
    def __init__(self):
        super().__init__('scan_restamper')
        self.clock_now = None
        self.map_odom_stamp = None
        self.sub_scan = self.create_subscription(
            LaserScan, '/scan_raw', self.on_scan, 10)
        self.sub_clock = self.create_subscription(
            Clock, '/clock', self.on_clock, 10)
        self.sub_tf = self.create_subscription(
            TFMessage, '/tf', self.on_tf, 10)
        self.pub = self.create_publisher(LaserScan, '/scan', 10)
        self.get_logger().info(
            're-stamping /scan_raw -> /scan at latest map->odom TF time')

    def on_clock(self, msg):
        self.clock_now = msg.clock

    def on_tf(self, msg):
        for t in msg.transforms:
            if t.header.frame_id == 'map' and t.child_frame_id == 'odom':
                self.map_odom_stamp = t.header.stamp

    def on_scan(self, msg):
        stamp = self.map_odom_stamp or self.clock_now
        if stamp is not None:
            msg.header.stamp = stamp
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ScanRestamper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
