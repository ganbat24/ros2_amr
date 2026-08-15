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


def _stamp_ns(stamp):
    """Convert a builtin_interfaces/Time message to nanoseconds."""
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def is_stale(clock_now, stamp):
    """
    Return True if `stamp` lags `clock_now` by more than STALE_AFTER_NS.

    Pure function over builtin_interfaces/Time messages (the exact types
    delivered on /clock and /tf). rclpy's Time class cannot subtract
    builtin Time messages directly, so the comparison is done in ns.
    """
    if clock_now is None:
        return False
    return _stamp_ns(clock_now) - _stamp_ns(stamp) > STALE_AFTER_NS


class TfRestamper(Node):
    """
    Re-stamps stale odom->base_link transforms, and reports how often.

    The counters exist so this node's necessity is a measurement rather than
    an assumption. Its sibling `scan_restamp` was carried for months on the
    belief that it was fixing dropped scans while it was in fact causing
    them; nothing here logged enough to tell the difference either way.

    If `restamped` stays at 0 across a full run, this node is provably inert
    on that host and should be removed rather than kept "just in case".
    """

    REPORT_PERIOD_S = 30.0

    def __init__(self):
        super().__init__('tf_restamper')
        self.clock_now = None
        self.seen = 0
        self.restamped = 0
        self.max_lag_ns = 0
        self.sub_tf = self.create_subscription(
            TFMessage, '/tf', self.on_tf, 10)
        self.sub_clock = self.create_subscription(
            Clock, '/clock', self.on_clock, 10)
        self.pub = self.create_publisher(TFMessage, '/tf', 10)
        # Wall-clock timer on purpose: this reports on the node's own
        # behaviour, and must keep reporting even if /clock stalls — which is
        # exactly the condition worth hearing about.
        self.timer = self.create_timer(self.REPORT_PERIOD_S, self.report)
        self.get_logger().info(
            're-stamping stale odom->base_link transforms at /clock time')

    def report(self):
        if not self.seen:
            self.get_logger().info('no odom->base_link transforms seen yet')
            return
        self.get_logger().info(
            'odom->base_link seen=%d restamped=%d (%.1f%%) max_lag=%.3f s'
            % (self.seen, self.restamped,
               100.0 * self.restamped / self.seen,
               self.max_lag_ns / 1e9))

    def on_clock(self, msg):
        self.clock_now = msg.clock

    def _is_stale(self, stamp):
        return is_stale(self.clock_now, stamp)

    def on_tf(self, msg):
        if self.clock_now is None:
            return
        republished = False
        for t in msg.transforms:
            if (t.header.frame_id != 'odom'
                    or t.child_frame_id != 'base_link'):
                continue
            # Count every candidate, not only the ones acted on: "seen 12000,
            # restamped 0" is the result that retires this node, and it is
            # indistinguishable from "never ran" without the denominator.
            #
            # This node's own republications land back on its subscription
            # (see the scope guard above), so when restamped > 0 each one
            # inflates `seen` by one and the percentage reads low. That does
            # not touch the decisive case: if restamped is 0 there are no
            # echoes, and `seen` is the true count.
            self.seen += 1
            lag = _stamp_ns(self.clock_now) - _stamp_ns(t.header.stamp)
            self.max_lag_ns = max(self.max_lag_ns, lag)
            if self._is_stale(t.header.stamp):
                t.header.stamp = self.clock_now
                self.restamped += 1
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
