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
# tf_restamp.py — re-stamp the EKF's odom->base_link /tf edge at current sim
# time and republish.
#
# Why: the diff_drive controller's clock (inside gz_ros_control) lags the
# /clock topic by seconds and drifts, so the EKF's odom->base_link transform
# carries stale stamps. Consumers (AMCL, Nav2 costmaps) then see every
# scan/TF as "earlier than all the data in the transform cache" and drop it.
# Re-stamping at /clock time aligns the TF stream with /clock and /scan.
#
# Scope guard (both rules are required):
# 1. Only the odom->base_link edge is re-stamped. map->odom (SLAM/AMCL) and
#    every other edge pass through untouched.
# 2. Only transforms that lag /clock by more than STALE_AFTER are re-stamped.
#    This node subscribes AND publishes on /tf; rclpy does not filter a
#    node's own publications back to its own subscriptions, so without the
#    age guard the node would re-publish its own output forever (each echo is
#    stamped at /clock time, i.e. fresh, so it is skipped by this guard and
#    the loop terminates).
import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from tf2_msgs.msg import TFMessage

# Far beyond DDS latency (ms), far below the observed controller-clock lag
# (seconds): the echo guard.
STALE_AFTER_NS = int(0.5e9)


class TfRestamper(Node):
    def __init__(self):
        super().__init__('tf_restamper')
        self.clock_now = None
        self.sub_tf = self.create_subscription(
            TFMessage, '/tf', self.on_tf, 10)
        self.sub_clock = self.create_subscription(
            Clock, '/clock', self.on_clock, 10)
        self.pub = self.create_publisher(TFMessage, '/tf', 10)
        self.get_logger().info(
            're-stamping stale odom->base_link transforms at /clock time')

    def on_clock(self, msg):
        self.clock_now = msg.clock

    def _is_stale(self, stamp):
        if self.clock_now is None:
            return False
        return (self.clock_now - stamp).nanoseconds > STALE_AFTER_NS

    def on_tf(self, msg):
        if self.clock_now is None:
            return
        republished = False
        for t in msg.transforms:
            if (t.header.frame_id == 'odom'
                    and t.child_frame_id == 'base_link'
                    and self._is_stale(t.header.stamp)):
                t.header.stamp = self.clock_now
                republished = True
        if republished:
            self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TfRestamper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
