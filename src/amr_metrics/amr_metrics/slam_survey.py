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
        self._last_stamp_ns = None
        self._frozen = 0

    def send(self, linear, angular):
        msg = self.msg_type()
        # Sim time: the node is created with use_sim_time, so its clock is
        # /clock. A wall-clock stamp here would look stale to the controller
        # and be discarded by its cmd_vel_timeout.
        stamp = self.node.get_clock().now().to_msg()
        # A frozen clock is the failure this guard exists for. It presents as
        # a robot that will not move while /cmd_vel_stamped publishes happily
        # at 20 Hz — the controller is silently discarding every message as
        # older than its own time. Naming it beats rediscovering it.
        as_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        if as_ns == self._last_stamp_ns:
            self._frozen += 1
            if self._frozen == 40:
                self.node.get_logger().error(
                    'command stamps are not advancing (%d.%09d). The node is '
                    'not being spun, so its /clock-driven time source is '
                    'frozen and diff_drive_controller will discard every '
                    'command as stale.' % (stamp.sec, stamp.nanosec))
        else:
            self._frozen = 0
            self._last_stamp_ns = as_ns
        msg.header.stamp = stamp
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = float(linear)
        msg.twist.angular.z = float(angular)
        self.pub.publish(msg)

    def stop(self, times=5):
        import rclpy
        for _ in range(times):
            self.send(0.0, 0.0)
            rclpy.spin_once(self.node, timeout_sec=0.05)


def drive_to(driver, poses, target, timeout, rate_hz=20.0):
    """Steer to (x, y). Returns (reached, distance_left).

    Spins the node each cycle rather than sleeping. With use_sim_time the
    node's clock is driven by its /clock subscription, and a node that is
    never spun has a frozen clock — every TwistStamped then carries the same
    stale stamp and diff_drive_controller discards it:

      Ignoring the received message (timestamp 132.49) because it is older
      than the current time by 25.84 seconds, which exceeds the allowed
      timeout (0.5000)

    The robot sits still while /cmd_vel_stamped publishes at 20 Hz and /odom
    reports 36 Hz, which looks like a drive-chain fault and is not one.
    Spinning also services the map subscription during the drive.
    """
    import rclpy
    tx, ty = target
    deadline = time.time() + timeout
    period = 1.0 / rate_hz
    while time.time() < deadline:
        rclpy.spin_once(driver.node, timeout_sec=0.0)
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
        rclpy.spin_once(driver.node, timeout_sec=period)
    driver.stop()
    with poses.lock:
        pose = poses.pose
    if pose is None:
        return False, float('inf')
    return False, math.hypot(tx - pose[2], ty - pose[3])


class MapCatcher:
    """Hold the latest /map, so saving it does not need a separate process.

    map_saver_cli is a short-lived process that has to discover a
    transient-local topic within its own timeout. On this host that failed
    twice in a row — once after a survey that drove 21/21 waypoints with
    slam_toolbox registering scans normally — and a mapping run that cannot
    save its map has produced nothing.

    Subscribing from the surveying node instead means the subscription is
    established before the drive starts and stays up throughout, which is the
    same reason this tool drives the controller directly rather than through
    nav2.
    """

    def __init__(self, node):
        from nav_msgs.msg import OccupancyGrid
        from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                               ReliabilityPolicy)
        self.grid = None
        # slam_toolbox latches /map: transient-local and reliable, depth 1.
        # A default sensor-data profile would never match it.
        qos = QoSProfile(depth=1,
                         reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         history=HistoryPolicy.KEEP_LAST)
        self.sub = node.create_subscription(
            OccupancyGrid, '/map', self._on_map, qos)

    def _on_map(self, msg):
        self.grid = msg

    def save(self, stem, offset=(0.0, 0.0)):
        """Write map_server-format PGM + YAML. Returns True if written.

        `offset` shifts the recorded origin into the world frame. It matters
        more than it looks: slam_toolbox origins its map at the robot's start
        pose, so the saved origin is relative to wherever the robot happened
        to begin. Scoring those coordinates against the simulated floorplan
        without the shift measures the offset rather than the map — measured
        2026-08-14, the same map scored 19.8% wall coverage and 0.618 m median
        error raw, against 92.5% and 0.005 m once shifted by the start pose.

        Writing it in world coordinates also makes the artifact directly
        comparable to the pre-built maps/amr_office.yaml, which is the point
        of producing it.
        """
        if self.grid is None:
            return False
        info = self.grid.info
        width, height = info.width, info.height
        # ROS occupancy: -1 unknown, 0..100 probability. map_server images
        # use 0 occupied, 254 free, 205 unknown, and store rows top-down
        # while the grid is row-major from the bottom-left — so rows are
        # emitted in reverse. Getting that backwards mirrors the map.
        rows = []
        for row in range(height - 1, -1, -1):
            start = row * width
            rows.append(bytes(
                0 if v >= 65 else (254 if 0 <= v <= 19 else 205)
                for v in self.grid.data[start:start + width]))
        with open(stem + '.pgm', 'wb') as handle:
            handle.write(b'P5\n%d %d\n255\n' % (width, height))
            handle.write(b''.join(rows))
        with open(stem + '.yaml', 'w') as handle:
            handle.write(
                'image: %s.pgm\n'
                'mode: trinary\n'
                'resolution: %f\n'
                'origin: [%f, %f, 0.0]\n'
                'negate: 0\n'
                'occupied_thresh: 0.65\n'
                'free_thresh: 0.196\n'
                % (os.path.basename(stem), info.resolution,
                   info.origin.position.x + offset[0],
                   info.origin.position.y + offset[1]))
        return True


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
        start_xy = (poses.pose[2], poses.pose[3])
        print('  start pose: (%.2f, %.2f)' % start_xy, flush=True)

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
    # Subscribe before driving, so the latch is captured no matter when
    # slam_toolbox first publishes.
    catcher = MapCatcher(node)
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
        # Spin here rather than sleeping: the map subscription only delivers
        # while the node is spun, and this is the only idle moment in the run.
        deadline = time.time() + 1.0
        while time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)

    driver.stop(times=10)
    # Give slam_toolbox a moment to publish its final map, spinning so the
    # subscription actually receives it.
    deadline = time.time() + 15.0
    while time.time() < deadline and catcher.grid is None:
        rclpy.spin_once(node, timeout_sec=0.2)
    for _ in range(20):
        rclpy.spin_once(node, timeout_sec=0.1)

    stem = os.path.join(args.out_dir, 'slam_map')
    if catcher.save(stem, offset=start_xy):
        print('  saved map -> %s.pgm/.yaml (%d x %d @ %.3f m)'
              % (stem, catcher.grid.info.width, catcher.grid.info.height,
                 catcher.grid.info.resolution), flush=True)
    else:
        print('  NO /map RECEIVED — slam_toolbox published nothing to save',
              flush=True)
    elapsed = time.time() - started
    print('== survey done: %d/%d waypoints in %.0f s =='
          % (reached, len(MAPPING_ROUTE), elapsed), flush=True)
    node.destroy_node()
    rclpy.shutdown()
    return 0 if reached == len(MAPPING_ROUTE) else 2


if __name__ == '__main__':
    sys.exit(main())
