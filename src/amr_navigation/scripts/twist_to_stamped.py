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
# twist_to_stamped.py — bridge the Nav2 velocity chain to ros2_control 4.x.
#
# nav2_velocity_smoother outputs plain geometry_msgs/Twist on
# /cmd_vel_smoothed, but ros2_controllers 4.x diff_drive subscribes
# TwistStamped on /cmd_vel. Republish stamped at /clock time.
from geometry_msgs.msg import Twist, TwistStamped
import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock


class TwistToStamped(Node):
    def __init__(self):
        super().__init__('twist_to_stamped')
        self.clock_now = None
        self.sub_twist = self.create_subscription(
            Twist, '/cmd_vel_smoothed', self.on_twist, 10)
        self.sub_clock = self.create_subscription(
            Clock, '/clock', self.on_clock, 10)
        self.pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.get_logger().info('Twist(/cmd_vel_smoothed) -> TwistStamped(/cmd_vel)')

    def on_clock(self, msg):
        self.clock_now = msg.clock

    def on_twist(self, msg):
        out = TwistStamped()
        if self.clock_now is not None:
            out.header.stamp = self.clock_now
        out.header.frame_id = 'odom'
        out.twist = msg
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = TwistToStamped()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
