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


def cancel_active_goal():
    """Cancel any in-flight navigate_to_pose goal server-side.

    `timeout N ros2 action send_goal ...` only SIGTERMs the CLI client on
    expiry — it does not cancel the goal on the action server, so bt_navigator
    keeps executing it in the background. That stale goal then gets silently
    preempted by whichever goal we send next, contaminating that goal's start
    position/costmap state with leftover motion from the "timed out" one.
    An empty CancelGoal request cancels all active goals on the server.
    """
    try:
        subprocess.run(
            ['bash', '-lc',
             'ros2 service call /navigate_to_pose/_action/cancel_goal '
             'action_msgs/srv/CancelGoal "{}"'],
            capture_output=True, text=True, timeout=10)
    except subprocess.TimeoutExpired:
        pass


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
    # UNKNOWN means the CLI was killed by `timeout` (or errored) before a
    # terminal status came back — the goal may still be executing server-side.
    cancel_active_goal()
    return 'UNKNOWN', wall


def settle():
    """Give the stack a beat before the next goal.

    Dispatching the next goal immediately after an ABORTED result was
    observed sending the new goal into a robot/BT that hadn't actually
    settled yet (still mid-recovery-behavior) — that goal then aborts almost
    instantly, which looks like a nav failure but is really a dispatch-timing
    artifact. (Tried also issuing an explicit clear-costmap service call here;
    dropped it — under load the request can sit in the node's callback queue
    for 20+ seconds behind a backlog from the goal that just finished, and it
    lands *after* the next goal's own planning call, adding to exactly the
    server-side congestion that causes bt_navigator's "timed out waiting for
    action server to acknowledge" abort. A plain wait avoids adding load.)
    """
    time.sleep(15)


def probe_ready(gate, retries, retry_wait):
    """Probe the drive chain; return True once odom shows real motion."""
    for i in range(retries):
        d = gate.probe()
        print('  probe %d: odom moved %.3f m' % (i + 1, d), flush=True)
        if d > 0.03:
            return True
        if i < retries - 1:
            print('  not ready; waiting %.0f s' % retry_wait, flush=True)
            time.sleep(retry_wait)
    return False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out-dir', default='/tmp/amr_validation')
    ap.add_argument('--goals', default='g1_top_right,g2_top_left,g3_bottom_right,g4_home')
    # 280.0 clipped at least one goal that eventually succeeded ~288 s in a
    # comparable attempt, so the harness gets headroom here. Raising it to
    # 400.0 was then measured on WSL2 and changed NOTHING: g1 ran the full
    # extended budget and still failed. That is the useful result — the
    # remaining failures are genuine stalls/oscillation, not slow-but-
    # progressing goals, so no timeout value rescues them. Keep the headroom
    # so the harness never mislabels a progressing goal, and look elsewhere
    # (sensor timing, DWB/costmap) for the actual failure.
    ap.add_argument('--per-goal-timeout', type=float, default=400.0)
    ap.add_argument('--record-duration', type=float, default=1500.0)
    ap.add_argument('--no-plot', action='store_true')
    args = ap.parse_args()

    import rclpy
    print('== drive-chain readiness gate ==')
    rclpy.init()
    gate = Gate()
    ok = probe_ready(gate, 5, 30)
    gate.destroy_node()
    rclpy.shutdown()
    if not ok:
        # Deliberately does NOT repair the stack. This tool measures; a tool
        # that restarts its own subject mid-run produces results nobody can
        # compare, which is how several earlier runs were confounded.
        print('DRIVE NEVER BECAME READY — aborting.', flush=True)
        print('This harness does not launch or repair the stack. Use:',
              flush=True)
        print('  ros2 run amr_metrics orchestrate --tour --out-dir <dir>',
              flush=True)
        return 1

    os.makedirs(args.out_dir, exist_ok=True)
    # Capture what this run ran on, so the numbers stay comparable later.
    try:
        from amr_metrics.orchestrate import capture_environment
        capture_environment(args.out_dir)
    except Exception as exc:                       # never block a run on this
        print('  (environment capture skipped: %s)' % exc, flush=True)
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
        settle()

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
