#!/usr/bin/env python3
"""Scripted validation tour: run a multi-goal navigation scenario, record
gz ground truth / odometry / AMCL, and produce the metrics report.

Flow (run inside the container, after sourcing the workspace):
  1. wait for the drive chain (ready_gate logic)
  2. start the trajectory recorder
  3. send the goal tour (default: g1 -> g2 -> g3 -> g4) via
     /navigate_to_pose; each goal gets its own timeout; the tour keeps
     going after a failed goal (recovery to the next)
  4. stop the recorder and plot the report

Usage:
  ros2 run amr_metrics run_validation [--out-dir DIR] [--goals g1,g2,g3,g4]
                                      [--per-goal-timeout 240] [--no-plot]
"""
import argparse
import os
import subprocess
import sys
import time

from amr_metrics.ready_gate import Gate
from amr_metrics import record_trajectory

GOAL_POSES = {
    'g1_top_right': (9.4, 7.4),
    'g2_top_left': (3.2, 6.8),
    'g3_bottom_right': (9.3, 1.5),
    'g4_home': (1.5, 1.5),
}


def send_goal(x, y, timeout_s):
    """Send a navigate_to_pose goal; return (status, wall_time)."""
    cmd = (
        'timeout %d ros2 action send_goal /navigate_to_pose '
        'nav2_msgs/action/NavigateToPose '
        '"{pose: {header: {frame_id: map}, pose: {position: {x: %f, y: %f, z: 0.0}, '
        'orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}"' % (timeout_s, x, y)
    )
    t0 = time.time()
    out = subprocess.run(['bash', '-lc', cmd], capture_output=True, text=True)
    wall = time.time() - t0
    full = out.stdout + out.stderr
    if 'SUCCEEDED' in full:
        return 'SUCCEEDED', wall
    if 'ABORTED' in full:
        return 'ABORTED', wall
    return 'UNKNOWN', wall


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out-dir', default='/tmp/amr_validation')
    ap.add_argument('--goals', default='g1_top_right,g2_top_left,g3_bottom_right,g4_home')
    ap.add_argument('--per-goal-timeout', type=float, default=280.0)
    ap.add_argument('--record-duration', type=float, default=1500.0)
    ap.add_argument('--no-plot', action='store_true')
    args = ap.parse_args()

    import rclpy
    rclpy.init()
    gate = Gate()
    print('== drive-chain readiness gate ==')
    ok = False
    for i in range(6):
        d = gate.probe()
        print('  probe %d: odom moved %.3f m' % (i + 1, d))
        if d > 0.03:
            ok = True
            break
        time.sleep(25)
    gate.destroy_node()
    rclpy.shutdown()
    if not ok:
        print('DRIVE NEVER BECAME READY — aborting')
        return 1

    os.makedirs(args.out_dir, exist_ok=True)
    csv_path = os.path.join(args.out_dir, 'traj.csv')
    goals = [g for g in args.goals.split(',') if g in GOAL_POSES]
    print('== recording to %s ==' % csv_path)
    rec = subprocess.Popen(
        ['python3', os.path.join(os.path.dirname(__file__), 'record_trajectory.py'),
         csv_path, '--duration', str(args.record_duration)],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(3)

    print('== goal tour: %s ==' % ' -> '.join(goals))
    results = {}
    for g in goals:
        x, y = GOAL_POSES[g]
        print('  -> %s (%.1f, %.1f)' % (g, x, y), flush=True)
        status, wall = send_goal(x, y, args.per_goal_timeout)
        results[g] = (status, wall)
        print('     %s in %.0f s wall' % (status, wall), flush=True)
        time.sleep(3)

    rec.terminate()
    rec.wait(timeout=10)
    print('== results ==')
    for g, (s, w) in results.items():
        print('  %-14s %-9s %.0f s' % (g, s, w))
    n_succ = sum(1 for s, _ in results.values() if s == 'SUCCEEDED')
    print('tour: %d/%d goals succeeded' % (n_succ, len(goals)))

    if not args.no_plot:
        out_png = os.path.join(args.out_dir, 'metrics_report.png')
        subprocess.run(['python3', os.path.join(os.path.dirname(__file__),
                                                'plot_metrics.py'), csv_path, out_png])
    return 0 if n_succ == len(goals) else 2


if __name__ == '__main__':
    sys.exit(main())
