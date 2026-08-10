#!/usr/bin/env python3
# clock_slow.py — republish the gz clock at 20 Hz on /clock.
#
# Why: the raw gz->ROS /clock bridge emits ~100 msgs/s with RELIABLE QoS,
# which backlogs the clock subscriptions of every node on a slow host
# (software rendering + 2 vCPU): their sim clocks drift seconds behind and
# the whole TF/scan timestamp alignment breaks ("earlier than all the
# data in the transform cache"). A 20 Hz /clock keeps every consumer's
# clock current with a fraction of the traffic.
import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock


class ClockSlow(Node):
    def __init__(self):
        super().__init__('clock_slow')
        self.latest = None
        self.sub = self.create_subscription(Clock, '/clock_raw', self.on_clock, 10)
        self.pub = self.create_publisher(Clock, '/clock', 10)
        self.timer = self.create_timer(0.05, self.on_timer)  # 20 Hz
        self.get_logger().info('republishing /clock_raw -> /clock at 20 Hz')

    def on_clock(self, msg):
        self.latest = msg.clock

    def on_timer(self):
        if self.latest is not None:
            out = Clock()
            out.clock = self.latest
            self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ClockSlow()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
