#!/usr/bin/env python3
# tf_restamp.py — re-stamp dynamic /tf (odom->base_link from the EKF) at
# current sim time and republish.
#
# Why: the diff_drive controller's clock (inside gz_ros_control) lags the
# /clock topic by several seconds and drifts, so the EKF's odom->base_link
# transform carries stale stamps. Consumers (AMCL, Nav2 costmaps) then see
# every scan/TF as "earlier than all the data in the transform cache" and
# drop it. Re-stamping at /clock time aligns the TF stream with /clock,
# /scan (see scan_restamp.py) and the consumers.
#
# NOTE: the transform CONTENT is the EKF's estimate at the controller's
# (lagged) time. For a stationary robot that is exact; for a moving robot
# it lags by the controller clock skew — a sim-environment artifact.
import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from rosgraph_msgs.msg import Clock


class TfRestamper(Node):
    def __init__(self):
        super().__init__('tf_restamper')
        self.clock_now = None
        self.sub_tf = self.create_subscription(
            TFMessage, '/tf', self.on_tf, 10)
        self.sub_clock = self.create_subscription(
            Clock, '/clock', self.on_clock, 10)
        self.pub = self.create_publisher(TFMessage, '/tf', 10)
        self.get_logger().info('re-stamping /tf at /clock time')

    def on_clock(self, msg):
        self.clock_now = msg.clock

    def on_tf(self, msg):
        if self.clock_now is not None:
            for t in msg.transforms:
                t.header.stamp = self.clock_now
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
