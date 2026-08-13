#!/usr/bin/env python3
"""Record gz ground truth + odometry + AMCL to CSV for metrics.

The ground truth comes from the Gazebo pose/info topic (parsed via the
``gz topic -e`` text-protobuf stream) because the ros_gz TFMessage bridge
drops model names. One CSV row every 0.2 s:
    wall_t, gt_x, gt_y, gt_yaw, odom_x, odom_y, odom_yaw, amcl_x, amcl_y
"""
import argparse
import csv
import math
import os
import signal
import subprocess
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from rosgraph_msgs.msg import Clock

GZ_TOPIC = '/world/amr_office/pose/info'
GZ_MODEL = 'amr'


class PoseParser:
    """Parse the gz pose/info text stream, tracking brace context so the
    position {x,y,z} is not confused with orientation {x,y,z,w}."""

    def __init__(self, name=GZ_MODEL, topic=GZ_TOPIC):
        self.name = name
        self.topic = topic
        self.lock = threading.Lock()
        self.pose = None  # (sec, nsec, x, y, yaw)

    def start(self):
        env = dict(os.environ)
        # Respect an existing GZ_IP (e.g. set by the launch stack for a
        # non-loopback gz_ip); only default to loopback if unset.
        env.setdefault('GZ_IP', '127.0.0.1')
        self.proc = subprocess.Popen(
            ['gz', 'topic', '-e', '-t', self.topic],
            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
            env=env, text=True, bufsize=1,
        )
        threading.Thread(target=self._read, daemon=True).start()

    def _read(self):
        cur = {}
        ctx = None  # 'position' | 'orientation'
        sec = nsec = 0
        for line in self.proc.stdout:
            line = line.strip()
            if line.startswith('sec:'):
                sec = int(line.split()[1])
            elif line.startswith('nsec:'):
                nsec = int(line.split()[1])
            elif line.startswith('name: "') and line.endswith('"'):
                cur = {'name': line.split('"')[1]}
            elif line.startswith('position {'):
                ctx = 'position'
            elif line.startswith('orientation {'):
                ctx = 'orientation'
            elif line == '}':
                if ctx == 'position':
                    cur['px'] = cur.get('x', 0.0)
                    cur['py'] = cur.get('y', 0.0)
                ctx = None
            elif line.startswith(('x:', 'y:', 'z:')) and ctx == 'position':
                cur[line[0]] = float(line.split()[1])
            elif line.startswith(('x:', 'y:', 'z:', 'w:')) and ctx == 'orientation':
                cur[line[0]] = float(line.split()[1])
                if line[0] == 'w' and cur.get('name') == self.name:
                    z = cur.get('z', 0.0)
                    yaw = math.atan2(2.0 * cur['w'] * z, 1.0 - 2.0 * z * z)
                    with self.lock:
                        self.pose = (sec, nsec, cur.get('px', 0.0),
                                     cur.get('py', 0.0), yaw)


class RecNode(Node):
    def __init__(self):
        super().__init__('metrics_recorder')
        self.odom = None
        self.amcl = None
        self.sim = None
        self.create_subscription(
            Odometry, '/odom', lambda m: setattr(self, 'odom', m), 10)
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose',
            lambda m: setattr(self, 'amcl', m), 10)
        self.create_subscription(
            Clock, '/clock', self._on_clock, 10)

    def _on_clock(self, m):
        self.sim = m.clock.sec + m.clock.nanosec * 1e-9


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('out', nargs='?', default='/tmp/traj.csv')
    ap.add_argument('--duration', type=float, default=600.0)
    ap.add_argument('--rate', type=float, default=5.0)
    args = ap.parse_args()

    stop = [False]
    signal.signal(signal.SIGTERM, lambda s, f: stop.__setitem__(0, True))
    signal.signal(signal.SIGINT, lambda s, f: stop.__setitem__(0, True))

    rclpy.init()
    node = RecNode()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    gt = PoseParser()
    gt.start()
    time.sleep(2)

    with open(args.out, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['wall_t', 'sim_t', 'gt_x', 'gt_y', 'gt_yaw',
                    'odom_x', 'odom_y', 'odom_yaw', 'amcl_x', 'amcl_y'])
        t0 = time.time()
        sim0 = node.sim if node.sim is not None else 0.0
        dt = 1.0 / args.rate
        while time.time() - t0 < args.duration and not stop[0]:
            with gt.lock:
                g = gt.pose
            o = node.odom
            a = node.amcl
            oyaw = math.atan2(
                2 * o.pose.pose.orientation.w * o.pose.pose.orientation.z,
                1 - 2 * o.pose.pose.orientation.z ** 2) if o else ''
            w.writerow([
                round(time.time() - t0, 2),
                round(node.sim - sim0, 2) if node.sim is not None else '',
                round(g[2], 4) if g else '', round(g[3], 4) if g else '',
                round(g[4], 4) if g else '',
                round(o.pose.pose.position.x, 4) if o else '',
                round(o.pose.pose.position.y, 4) if o else '',
                round(oyaw, 4) if o else '',
                round(a.pose.pose.position.x, 4) if a else '',
                round(a.pose.pose.position.y, 4) if a else '',
            ])
            f.flush()
            time.sleep(dt)
    node.destroy_node()
    rclpy.shutdown()
    print('wrote %s' % args.out)


if __name__ == '__main__':
    main()
