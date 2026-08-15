#!/usr/bin/env python3
"""Drive a continuous waypoint route, optionally for many laps.

The four-goal tour answers "can it reach a pose". It cannot answer "can it
keep running", because every leg starts from a stationary robot with a freshly
settled stack and 15 s of deliberate settling in between. Nothing in this
project has ever measured what happens after twenty minutes of continuous
motion — whether localisation drifts, whether costmaps accumulate stale marks,
whether success rate decays.

This sends one /follow_waypoints goal with number_of_loops set, so nav2 drives
the route continuously, and then reports whether the run got *worse over
time*: AMCL error is bucketed by lap, so a degrading run is visible as a trend
rather than hidden inside a single median.

Laps are counted from feedback rather than assumed. nav2's number_of_loops is
documented nowhere in a way worth trusting, so the run reports both what was
requested and what was observed, and a mismatch is printed rather than
smoothed over.

Usage:
  ros2 run amr_metrics run_waypoints --loops 4 --out-dir /tmp/autonomy
"""
import argparse
import json
import math
import os
import statistics
import subprocess
import sys
import time

from amr_metrics.ready_gate import Gate
from amr_metrics.run_validation import GOAL_POSES

DEFAULT_ROUTE = 'g1_top_right,g2_top_left,g3_bottom_right,g4_home'


def make_pose(x, y):
    from geometry_msgs.msg import PoseStamped
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.orientation.w = 1.0
    return pose


class LapCounter:
    """Counts route laps from the action's current_waypoint feedback.

    The index climbs 0..N-1 then restarts at 0 for the next loop, so a
    decrease marks a lap boundary. Timestamps are wall clock, which is what
    "did it get slower" means for an operator.
    """

    def __init__(self):
        self.previous = None
        self.boundaries = [time.time()]

    def update(self, current):
        if self.previous is not None and current < self.previous:
            self.boundaries.append(time.time())
        self.previous = current

    def lap_times(self):
        edges = self.boundaries + [time.time()]
        return [edges[i + 1] - edges[i] for i in range(len(edges) - 1)]


def amcl_error_by_lap(csv_path, boundaries, start_wall):
    """Median AMCL error per lap, to expose drift that a single median hides.

    Lap boundaries are wall-clock; the CSV's wall_t is relative to when the
    recorder started, so they are compared through start_wall.
    """
    rows = []
    with open(csv_path) as handle:
        header = handle.readline().strip().split(',')
        idx = {name: i for i, name in enumerate(header)}
        for line in handle:
            parts = line.strip().split(',')
            if len(parts) != len(header):
                continue
            try:
                if not parts[idx['amcl_x']] or not parts[idx['gt_x']]:
                    continue
                rows.append((
                    float(parts[idx['wall_t']]),
                    math.hypot(
                        float(parts[idx['amcl_x']]) - float(parts[idx['gt_x']]),
                        float(parts[idx['amcl_y']]) - float(parts[idx['gt_y']])),
                ))
            except (ValueError, KeyError):
                continue

    if not rows:
        return []
    relative = [b - start_wall for b in boundaries] + [float('inf')]
    per_lap = []
    for i in range(len(relative) - 1):
        errors = [e for t, e in rows if relative[i] <= t < relative[i + 1]]
        per_lap.append({
            'lap': i + 1,
            'samples': len(errors),
            'median_error_m': round(statistics.median(errors), 4) if errors else None,
            'max_error_m': round(max(errors), 4) if errors else None,
        })
    return per_lap


def follow(route_poses, loops, timeout_s):
    """Send the FollowWaypoints goal. Returns (status, result, counter)."""
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.node import Node as RclpyNode
    from nav2_msgs.action import FollowWaypoints

    node = RclpyNode('waypoint_runner')
    client = ActionClient(node, FollowWaypoints, 'follow_waypoints')
    if not client.wait_for_server(timeout_sec=30.0):
        node.destroy_node()
        return 'NO_SERVER', None, None

    goal = FollowWaypoints.Goal()
    goal.poses = [make_pose(x, y) for x, y in route_poses]
    goal.number_of_loops = int(loops)

    counter = LapCounter()

    def on_feedback(msg):
        counter.update(msg.feedback.current_waypoint)

    send = client.send_goal_async(goal, feedback_callback=on_feedback)
    rclpy.spin_until_future_complete(node, send, timeout_sec=60.0)
    handle = send.result()
    if handle is None or not handle.accepted:
        node.destroy_node()
        return 'REJECTED', None, counter

    result_future = handle.get_result_async()
    deadline = time.time() + timeout_s
    while rclpy.ok() and not result_future.done():
        rclpy.spin_once(node, timeout_sec=1.0)
        if time.time() > deadline:
            handle.cancel_goal_async()
            rclpy.spin_once(node, timeout_sec=5.0)
            node.destroy_node()
            return 'TIMEOUT', None, counter

    wrapped = result_future.result()
    node.destroy_node()
    return ('SUCCEEDED' if wrapped.status == 4 else 'FAILED_%d' % wrapped.status,
            wrapped.result, counter)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--out-dir', default='/tmp/amr_autonomy')
    ap.add_argument('--route', default=DEFAULT_ROUTE)
    ap.add_argument('--loops', type=int, default=0,
                    help='extra repeats of the route beyond the first pass; '
                         'nav2 semantics, reported against observed laps')
    ap.add_argument('--timeout', type=float, default=3000.0)
    ap.add_argument('--record-duration', type=float, default=4000.0)
    ap.add_argument('--no-plot', action='store_true')
    args = ap.parse_args()

    names = [g for g in args.route.split(',') if g in GOAL_POSES]
    if not names:
        print('no valid goals in --route %r' % args.route)
        return 1
    route = [GOAL_POSES[g] for g in names]

    import rclpy
    print('== drive-chain readiness gate ==')
    rclpy.init()
    gate = Gate()
    moved = 0.0
    for attempt in range(5):
        moved = gate.probe()
        print('  probe %d: odom moved %.3f m' % (attempt + 1, moved), flush=True)
        if moved > 0.03:
            break
        time.sleep(30)
    gate.destroy_node()
    if moved <= 0.03:
        # Same contract as run_validation: measure, never repair.
        print('DRIVE NEVER BECAME READY — aborting.', flush=True)
        print('Use: ros2 run amr_metrics orchestrate --launch --wait-ready',
              flush=True)
        rclpy.shutdown()
        return 1

    os.makedirs(args.out_dir, exist_ok=True)
    try:
        from amr_metrics.orchestrate import capture_environment
        capture_environment(args.out_dir)
    except Exception as exc:
        print('  (environment capture skipped: %s)' % exc, flush=True)

    csv_path = os.path.join(args.out_dir, 'traj.csv')
    print('== recording to %s ==' % csv_path)
    # start_wall must be the recorder's own t=0, not the moment the route is
    # dispatched: the CSV's wall_t is relative to when the recorder started,
    # and lap boundaries are absolute. Taking it after the settle below would
    # shift every lap bucket by that settle, quietly attributing the first
    # seconds of each lap to the previous one.
    start_wall = time.time()
    recorder = subprocess.Popen(
        ['python3', os.path.join(os.path.dirname(__file__),
                                 'record_trajectory.py'),
         csv_path, '--duration', str(args.record_duration)],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(3)

    print('== waypoint route: %s (%d extra loop(s)) =='
          % (' -> '.join(names), args.loops), flush=True)
    route_start = time.time()
    status, result, counter = follow(route, args.loops, args.timeout)
    elapsed = time.time() - route_start

    recorder.terminate()
    try:
        recorder.wait(timeout=10)
    except subprocess.TimeoutExpired:
        recorder.kill()

    missed = []
    if result is not None:
        missed = [{'index': int(m.index), 'error_code': int(m.error_code)}
                  for m in result.missed_waypoints]

    lap_times = counter.lap_times() if counter else []
    observed_laps = len(lap_times)
    per_lap = amcl_error_by_lap(csv_path, counter.boundaries,
                                start_wall) if counter else []

    print('== results ==')
    print('  status            %s' % status)
    print('  wall time         %.0f s' % elapsed)
    print('  laps requested    %d' % (args.loops + 1))
    print('  laps observed     %d' % observed_laps)
    if observed_laps != args.loops + 1:
        print('    ! requested and observed lap counts differ — nav2 counts '
              'number_of_loops as repeats beyond the first pass, and this '
              'run did not match that reading')
    print('  waypoints missed  %d of %d'
          % (len(missed), len(route) * max(observed_laps, 1)))
    for entry in missed:
        print('    index %d (error_code %d)'
              % (entry['index'], entry['error_code']))
    if lap_times:
        print('  lap times         %s'
              % ', '.join('%.0f s' % t for t in lap_times))
    for entry in per_lap:
        print('  lap %d AMCL error median %s max %s (%d samples)'
              % (entry['lap'],
                 '%.3f m' % entry['median_error_m']
                 if entry['median_error_m'] is not None else 'n/a',
                 '%.3f m' % entry['max_error_m']
                 if entry['max_error_m'] is not None else 'n/a',
                 entry['samples']))

    with open(os.path.join(args.out_dir, 'waypoint_results.json'), 'w') as fh:
        json.dump({
            'route': names,
            'status': status,
            'wall_s': round(elapsed, 1),
            'loops_requested': args.loops,
            'laps_observed': observed_laps,
            'lap_times_s': [round(t, 1) for t in lap_times],
            'missed_waypoints': missed,
            'amcl_error_by_lap': per_lap,
        }, fh, indent=2)

    if not args.no_plot:
        subprocess.run([
            'python3',
            os.path.join(os.path.dirname(__file__), 'plot_metrics.py'),
            csv_path, os.path.join(args.out_dir, 'metrics_report.png')])

    rclpy.shutdown()
    return 0 if status == 'SUCCEEDED' and not missed else 2


if __name__ == '__main__':
    sys.exit(main())
