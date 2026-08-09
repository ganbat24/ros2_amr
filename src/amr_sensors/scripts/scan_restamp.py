#!/usr/bin/env python3
# scan_restamp.py — re-stamp /scan_raw at current sim time and republish as /scan.
#
# Why: under headless software rendering (llvmpipe) the gpu_lidar's render
# thread runs seconds behind the physics clock, so raw scan messages carry
# stale timestamps and consumers (AMCL, SLAM, costmaps) drop them against
# the TF cache. We re-stamp from the /clock topic so scan stamps match the
# consumers' sim-clock time base.
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rosgraph_msgs.msg import Clock


class ScanRestamper(Node):
    def __init__(self):
        super().__init__('scan_restamper')
        self.clock_now = None
        self.sub_scan = self.create_subscription(
            LaserScan, '/scan_raw', self.on_scan, 10)
        self.sub_clock = self.create_subscription(
            Clock, '/clock', self.on_clock, 10)
        self.pub = self.create_publisher(LaserScan, '/scan', 10)
        self.get_logger().info('re-stamping /scan_raw -> /scan at /clock time')

    def on_clock(self, msg):
        self.clock_now = msg.clock

    def on_scan(self, msg):
        if self.clock_now is not None:
            msg.header.stamp = self.clock_now
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
