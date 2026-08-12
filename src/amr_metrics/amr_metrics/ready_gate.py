#!/usr/bin/env python3
"""Drive-chain readiness gate.

The gz_ros2_control <-> physics bridge needs ~60-90 s to warm up after a
fresh launch on constrained hosts: until then the odometry feedback is
frozen even though the command path works, and navigation started in this
window drives blind (the O1-wedge class of failure). This node commands a
short velocity probe and waits until /odom responds before returning.

Usage (after sourcing the workspace):
    ros2 run amr_metrics ready_gate [--retries N] [--retry-wait S]
Exit code 0 = drive chain live, 1 = never became ready.
"""
import argparse
import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

PROBE_VX = 0.1   # m/s
PROBE_DT = 2.0   # s
MIN_DELTA = 0.03  # m of odom motion that proves the feedback is live


class Gate(Node):
    def __init__(self):
        super().__init__('ready_gate')
        self.odom = None
        self.create_subscription(Odometry, '/odom',
                                 lambda m: setattr(self, 'odom', m), 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def probe(self):
        """Command PROBE_VX for PROBE_DT s; return odom motion magnitude."""
        before = self.odom
        t0 = time.time()
        while time.time() - t0 < 0.5 and before is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        before = self.odom
        if before is None:
            self.get_logger().warn('no /odom received yet')
            return 0.0
        bx, by = before.pose.pose.position.x, before.pose.pose.position.y
        msg = Twist()
        msg.linear.x = PROBE_VX
        end = time.time() + PROBE_DT
        while time.time() < end:
            self.pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.05)
        stop = Twist()
        for _ in range(5):
            self.pub.publish(stop)
            time.sleep(0.05)
        # sample the odom after the command stops
        time.sleep(1.0)
        rclpy.spin_once(self, timeout_sec=1.0)
        o = self.odom
        if o is None:
            return 0.0
        return math.hypot(o.pose.pose.position.x - bx,
                          o.pose.pose.position.y - by)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--retries', type=int, default=8)
    ap.add_argument('--retry-wait', type=float, default=30.0)
    args = ap.parse_args()

    rclpy.init()
    node = Gate()
    ok = False
    for i in range(args.retries):
        d = node.probe()
        node.get_logger().info('ready probe %d: odom moved %.3f m' % (i + 1, d))
        if d > MIN_DELTA:
            ok = True
            break
        if i < args.retries - 1:
            node.get_logger().info('not ready; waiting %.0f s' % args.retry_wait)
            time.sleep(args.retry_wait)
    node.destroy_node()
    rclpy.shutdown()
    print('DRIVE READY' if ok else 'DRIVE NEVER BECAME READY')
    raise SystemExit(0 if ok else 1)


if __name__ == '__main__':
    main()
