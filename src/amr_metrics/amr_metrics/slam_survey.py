#!/usr/bin/env python3
"""Drive the mapping route directly, without nav2, and build a SLAM map.

Mapping does not need a navigation stack, and driving through one adds a
dependency that has repeatedly not been up in time. Under `use_slam:=true`
nav2's lifecycle manager was seen with `/velocity_smoother` stuck at
`unconfigured` for a 240 s timeout; the cause was upstream of nav2 — the EKF
had not yet acquired /clock, so `odom -> base_link` did not exist and the
costmaps could not activate. Mapping through the goal tour fails for an
unrelated reason as well: it dispatches to coordinates 8 m away in a world
slam_toolbox has not seen, so the planner has nothing to plan through, and it
scored 1/4.

So this drives the wheels itself. It publishes TwistStamped straight to
/cmd_vel_stamped, which is what diff_drive_controller subscribes to, bypassing
velocity_smoother and twist_to_stamped along with the whole nav2 stack. The
only things that need to be up are Gazebo, the controllers and slam_toolbox.

Pose comes from Gazebo ground truth rather than odometry. That is deliberate
and it is not cheating: the mission is to move the robot over a known-clear
route so the lidar sees the whole floor. Using drifting odometry to steer
would put the robot into walls for reasons that have nothing to do with the
map being measured.

Usage:
  ros2 run amr_metrics slam_survey --out-dir /tmp/slam_run
"""
import argparse
import math
import os
import sys
import time

from amr_metrics.record_trajectory import PoseParser
from amr_metrics.run_waypoints import MAPPING_ROUTE

# Deliberately gentle. The point is a clean, well-registered scan sequence,
# not a fast lap: slam_toolbox matches scans against its graph, and a robot
# that spins quickly produces scans it cannot register.
MAX_LIN = 0.35
MAX_ANG = 0.8
ARRIVE_TOL = 0.25
# Above this heading error, turn in place rather than arc — arcing toward a
# target that is behind the robot sweeps a wide curve into whatever is beside
# it, and the route's clearances assume roughly straight segments.
TURN_IN_PLACE = 0.6


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def clamp(value, limit):
    return max(-limit, min(limit, value))


class Driver:
    def __init__(self, node):
        from geometry_msgs.msg import TwistStamped
        self.node = node
        self.msg_type = TwistStamped
        self.pub = node.create_publisher(TwistStamped, '/cmd_vel_stamped', 10)

    def send(self, linear, angular):
        msg = self.msg_type()
        # Sim time: the node is created with use_sim_time, so its clock is
        # /clock. A wall-clock stamp here would look stale to the controller
        # and be discarded by its cmd_vel_timeout.
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = float(linear)
        msg.twist.angular.z = float(angular)
        self.pub.publish(msg)

    def stop(self, times=5):
        for _ in range(times):
            self.send(0.0, 0.0)
            time.sleep(0.05)


def drive_to(driver, poses, target, timeout, rate_hz=20.0):
    """Steer to (x, y). Returns (reached, distance_left)."""
    tx, ty = target
    deadline = time.time() + timeout
    period = 1.0 / rate_hz
    while time.time() < deadline:
        with poses.lock:
            pose = poses.pose
        if pose is None:
            time.sleep(period)
            continue
        _, _, x, y, yaw = pose
        dx, dy = tx - x, ty - y
        dist = math.hypot(dx, dy)
        if dist <= ARRIVE_TOL:
            driver.stop()
            return True, dist
        heading_error = wrap(math.atan2(dy, dx) - yaw)
        if abs(heading_error) > TURN_IN_PLACE:
            driver.send(0.0, clamp(1.5 * heading_error, MAX_ANG))
        else:
            driver.send(clamp(0.9 * dist, MAX_LIN),
                        clamp(1.2 * heading_error, MAX_ANG))
        time.sleep(period)
    driver.stop()
    with poses.lock:
        pose = poses.pose
    if pose is None:
        return False, float('inf')
    return False, math.hypot(tx - pose[2], ty - pose[3])


def wait_for_odom_tf(node, timeout):
    """Block until odom -> base_link exists. Returns True if it appeared.

    slam_toolbox cannot map without this transform: with no odom frame its
    message filter drops every scan with "the timestamp on the message is
    earlier than all the data in the transform cache" — the cache being empty
    because the frame does not exist. Measured 2026-08-14: the EKF hung at
    "Waiting for clock to start..." while /clock published at 40 Hz, and this
    survey then drove a flawless 21/21 route that produced 74 dropped scans
    and no map at all.

    Checking it here turns six wasted minutes into a fast failure that
    orchestrate can retry, which is the whole value of a readiness gate.
    """
    import rclpy
    from tf2_ros import Buffer, TransformListener

    buffer = Buffer()
    listener = TransformListener(buffer, node)
    deadline = time.time() + timeout
    try:
        while time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.5)
            if buffer.can_transform('odom', 'base_link',
                                    rclpy.time.Time()):
                return True
        return False
    finally:
        del listener


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--out-dir', default='/tmp/amr_slam_survey')
    ap.add_argument('--per-waypoint-timeout', type=float, default=90.0)
    ap.add_argument('--pose-wait', type=float, default=60.0)
    # Short by design. orchestrate's gate is the retryable check and waits
    # much longer; this is a last-line assertion so a survey run standalone
    # still refuses to drive blind.
    ap.add_argument('--tf-wait', type=float, default=20.0,
                    help='seconds to wait for odom->base_link before giving '
                         'up; without it slam_toolbox drops every scan')
    args = ap.parse_args()

    import rclpy
    from rclpy.node import Node

    rclpy.init()
    node = Node('slam_survey',
                parameter_overrides=[
                    rclpy.parameter.Parameter('use_sim_time', value=True)])

    poses = PoseParser()
    poses.start()
    print('== waiting for Gazebo ground truth ==', flush=True)
    deadline = time.time() + args.pose_wait
    while time.time() < deadline:
        with poses.lock:
            if poses.pose is not None:
                break
        time.sleep(0.5)
    with poses.lock:
        if poses.pose is None:
            print('NO GROUND TRUTH POSE — is Gazebo up? aborting.', flush=True)
            rclpy.shutdown()
            return 1
        print('  start pose: (%.2f, %.2f)' % (poses.pose[2], poses.pose[3]),
              flush=True)

    print('== waiting for odom -> base_link (slam_toolbox needs it) ==',
          flush=True)
    if not wait_for_odom_tf(node, args.tf_wait):
        print('NO odom -> base_link TRANSFORM after %.0f s — aborting before '
              'driving.' % args.tf_wait, flush=True)
        print('  The EKF is not publishing it. Check the launch log for '
              '"[ekf_filter_node]: Waiting for clock to start..." — that is '
              'the EKF failing to discover /clock, and slam_toolbox will drop '
              'every scan while it persists.', flush=True)
        rclpy.shutdown()
        return 1
    print('  odom -> base_link present', flush=True)

    os.makedirs(args.out_dir, exist_ok=True)
    driver = Driver(node)

    print('== driving %d mapping waypoints ==' % len(MAPPING_ROUTE), flush=True)
    reached = 0
    started = time.time()
    for index, target in enumerate(MAPPING_ROUTE, 1):
        ok, left = drive_to(driver, poses, target, args.per_waypoint_timeout)
        reached += 1 if ok else 0
        print('  wp%02d (%.1f, %.1f) %s  %.2f m left'
              % (index, target[0], target[1],
                 'reached' if ok else 'TIMEOUT', left), flush=True)
        # Pause so slam_toolbox gets a stationary scan at each vertex; scans
        # taken mid-turn are the ones it fails to register.
        driver.stop()
        time.sleep(1.0)

    driver.stop(times=10)
    elapsed = time.time() - started
    print('== survey done: %d/%d waypoints in %.0f s =='
          % (reached, len(MAPPING_ROUTE), elapsed), flush=True)
    node.destroy_node()
    rclpy.shutdown()
    return 0 if reached == len(MAPPING_ROUTE) else 2


if __name__ == '__main__':
    sys.exit(main())
