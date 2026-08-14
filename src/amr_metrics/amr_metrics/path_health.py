#!/usr/bin/env python3
"""Is the global plan stable, or is it flip-flopping between routes.

DWB scores every candidate trajectory against the current global plan. If that
plan changes materially between control cycles, the alignment critics move
under the controller's feet and the lowest-cost choice becomes "barely move" —
a robot that creeps while every component reports healthy.

This measures how much the plan actually changes between replans:

  stable    : successive plans overlap closely. Alignment critics are
              meaningful and a crawling robot is not their fault.
  drifting  : small changes each replan, normal near obstacles.
  flip-flop : large jumps, i.e. the planner is alternating between routes of
              near-equal cost. Fix the planner's cost landscape
              (cost_travel_multiplier, inflation) before touching the
              controller.

Usage (needs a running stack, mid-navigation):
    ros2 run amr_metrics path_health --duration 60
"""
import argparse
import math
import statistics
import sys
import time


def _resample(points, n=25):
    """Resample a polyline to n evenly spaced points for fair comparison."""
    if len(points) < 2:
        return points * n
    seg = [0.0]
    for a, b in zip(points, points[1:]):
        seg.append(seg[-1] + math.dist(a, b))
    total = seg[-1]
    if total == 0:
        return [points[0]] * n
    out = []
    for i in range(n):
        target = total * i / (n - 1)
        j = max(k for k in range(len(seg)) if seg[k] <= target)
        if j >= len(points) - 1:
            out.append(points[-1])
            continue
        span = seg[j + 1] - seg[j]
        f = 0.0 if span == 0 else (target - seg[j]) / span
        ax, ay = points[j]
        bx, by = points[j + 1]
        out.append((ax + f * (bx - ax), ay + f * (by - ay)))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--duration', type=float, default=60.0)
    ap.add_argument('--topic', default='/plan')
    args = ap.parse_args()

    import rclpy
    from rclpy.node import Node
    from nav_msgs.msg import Path

    class PathHealth(Node):
        def __init__(self):
            super().__init__('path_health')
            self.set_parameters([
                rclpy.parameter.Parameter('use_sim_time',
                                          rclpy.Parameter.Type.BOOL, True)])
            self.plans = []
            self.create_subscription(Path, args.topic, self._on_plan, 10)

        def _on_plan(self, msg):
            pts = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
            if pts:
                self.plans.append((time.time(), pts))

    rclpy.init()
    node = PathHealth()
    print('watching %s for %.0f s ...' % (args.topic, args.duration),
          flush=True)
    end = time.time() + args.duration
    while time.time() < end and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.2)

    n = len(node.plans)
    if n < 2:
        print('\nRESULT: %d plans received — the planner is not publishing.'
              % n)
        node.destroy_node()
        rclpy.shutdown()
        return 1

    rate = n / args.duration
    lengths = []
    for _, pts in node.plans:
        lengths.append(sum(math.dist(a, b) for a, b in zip(pts, pts[1:])))

    # Compare consecutive plans after resampling: mean point-to-point shift.
    shifts = []
    for (_, a), (_, b) in zip(node.plans, node.plans[1:]):
        ra, rb = _resample(a), _resample(b)
        shifts.append(statistics.mean(math.dist(p, q) for p, q in zip(ra, rb)))

    print('\n=== global plan over %.0f s ===' % args.duration)
    print('  plans published     %d  (%.2f Hz)' % (n, rate))
    print('  plan length         median %.2f m  min %.2f  max %.2f'
          % (statistics.median(lengths), min(lengths), max(lengths)))
    print('  change between plans  median %.3f m  max %.3f m'
          % (statistics.median(shifts), max(shifts)))
    big = [s for s in shifts if s > 0.5]
    print('  replans shifting >0.5 m   %d / %d  (%.0f%%)'
          % (len(big), len(shifts), 100 * len(big) / len(shifts)))

    print('\n=== verdict ===')
    med = statistics.median(shifts)
    if med > 0.5 or len(big) / len(shifts) > 0.3:
        print('  * FLIP-FLOPPING: the plan moves %.2f m between replans.'
              % med)
        print('    The controller is chasing a moving target, so its path')
        print('    alignment critics never settle. Fix the planner cost')
        print('    landscape (cost_travel_multiplier, inflation_radius)')
        print('    before tuning the controller.')
    elif med > 0.15:
        print('  * DRIFTING: %.2f m median change. Normal near obstacles;'
              % med)
        print('    worth a look if the robot also crawls.')
    else:
        print('  Plan is stable (%.3f m median change). If the robot still'
              % med)
        print('  crawls, the cause is in the controller, not the planner.')

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
