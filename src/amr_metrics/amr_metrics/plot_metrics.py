#!/usr/bin/env python3
"""Plot validation metrics from a trajectory CSV.

Reads the CSV written by record_trajectory.py and produces a PNG report:
  - trajectory overlay (gz truth vs odometry vs AMCL) on the office plan
  - position error vs time: odometry drift (relative to run start) and
    AMCL localization error vs ground truth
  - yaw error vs time (odometry and AMCL vs truth)
  - speed profile from ground truth

Usage: ros2 run amr_metrics plot_metrics <traj.csv> [out.png]
"""
import argparse
import math
import os
import re
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

# Fallback floorplan (only used when the generated world SDF cannot be found)
_FALLBACK_WALLS = [
    (0.0, 0.0, 10.0, 0.2), (0.0, 7.8, 10.0, 8.0), (0.0, 0.0, 0.2, 8.0),
    (9.8, 0.0, 10.0, 8.0),
    (0.0, 3.9, 4.4, 4.1), (6.6, 3.9, 10.0, 4.1),
    (7.7, 4.1, 7.9, 6.0),
]
_FALLBACK_OBSTACLES = [
    (3.1, 1.6, 3.9, 2.4), (8.15, 1.65, 8.85, 2.35), (1.5, 6.3, 2.1, 6.9),
    (8.4, 4.6, 9.1, 5.3),
]
SPAWN = (1.5, 1.5)
GOALS = {'g1_top_right': (9.4, 7.4), 'g2_top_left': (3.2, 6.8),
         'g3_bottom_right': (9.3, 1.5), 'g4_home': (1.5, 1.5)}


def _world_sdf_candidates():
    """Locate the generated world SDF (the single source of truth for the
    floorplan): ament share first, then the source tree."""
    cands = []
    try:
        from ament_index_python.packages import get_package_share_directory
        try:
            share = get_package_share_directory('amr_simulation')
            cands.append(os.path.join(share, 'worlds', 'amr_office.sdf'))
        except Exception:
            pass
    except ImportError:
        pass
    here = os.path.dirname(os.path.abspath(__file__))
    cands += [
        os.path.normpath(os.path.join(
            here, '..', '..', 'amr_simulation', 'worlds', 'amr_office.sdf')),
        '/ros2_ws/src/amr_simulation/worlds/amr_office.sdf',
    ]
    return cands


def load_floorplan():
    """Parse wall_*/obstacle_* model boxes out of the generated world SDF.

    Returns (walls, obstacles) as (xmin, ymin, xmax, ymax) tuples, so the
    report always shows the floorplan that was actually simulated.
    """
    sdf = None
    for p in _world_sdf_candidates():
        if os.path.isfile(p):
            with open(p) as f:
                sdf = f.read()
            break
    if sdf is None:
        print('WARNING: amr_office.sdf not found, using fallback floorplan')
        return _FALLBACK_WALLS, _FALLBACK_OBSTACLES

    walls, obstacles = [], []
    for m in re.finditer(
            r'<model name="(wall_\d+|obstacle_\d+)">(.*?)</model>',
            sdf, re.S):
        name, body = m.group(1), m.group(2)
        pose = re.search(r'<pose>\s*([-\d.eE]+)\s+([-\d.eE]+)', body)
        size = re.search(r'<size>\s*([-\d.eE]+)\s+([-\d.eE]+)', body)
        if not pose or not size:
            continue
        cx, cy = float(pose.group(1)), float(pose.group(2))
        sx, sy = float(size.group(1)), float(size.group(2))
        rect = (cx - sx / 2, cy - sy / 2, cx + sx / 2, cy + sy / 2)
        (walls if name.startswith('wall') else obstacles).append(rect)
    return walls, obstacles


WALLS, OBSTACLES = load_floorplan()


def load_csv(path):
    rows = []
    with open(path) as f:
        header = f.readline().strip().split(',')
        for line in f:
            parts = line.strip().split(',')
            if len(parts) != len(header):
                continue
            r = {}
            for h, v in zip(header, parts):
                try:
                    r[h] = float(v)
                except ValueError:
                    r[h] = None
            rows.append(r)
    return rows


def series(rows, key):
    return np.array([r[key] if r[key] is not None else np.nan for r in rows])


def wrap(yaw):
    return (yaw + np.pi) % (2 * np.pi) - np.pi


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('csv')
    ap.add_argument('out', nargs='?', default='metrics_report.png')
    args = ap.parse_args()

    rows = load_csv(args.csv)
    if len(rows) < 10:
        print('too few rows (%d)' % len(rows))
        return 1
    # x-axis: simulation time when recorded, else wall time
    if 'sim_t' in rows[0]:
        t = series(rows, 'sim_t')
        tunit = 'sim time'
    else:
        t = series(rows, 'wall_t')
        tunit = 'wall time'
    gx, gy = series(rows, 'gt_x'), series(rows, 'gt_y')
    odo_x, odo_y = series(rows, 'odom_x'), series(rows, 'odom_y')
    ax_, ay_ = series(rows, 'amcl_x'), series(rows, 'amcl_y')
    gt_yaw = series(rows, 'gt_yaw')

    def _yaw(col):
        v = series(rows, col)
        return v

    # odometry drift error: relative to the run start (odom frame anchor
    # is unknown/arbitrary, so compare deltas)
    valid_o = ~np.isnan(odo_x)
    drift = np.full_like(t, np.nan)
    if valid_o.sum() > 2:
        i0 = np.argmax(valid_o)
        dx = odo_x - odo_x[i0]
        dy = odo_y - odo_y[i0]
        dgx = gx - gx[i0]
        dgy = gy - gy[i0]
        drift = np.hypot(dx - dgx, dy - dgy)
        drift[~valid_o] = np.nan

    # AMCL localization error vs truth
    valid_a = ~np.isnan(ax_)
    amcl_err = np.full_like(t, np.nan)
    if valid_a.sum() > 2:
        amcl_err = np.hypot(ax_ - gx, ay_ - gy)
        amcl_err[~valid_a] = np.nan

    # yaw errors (odometry vs truth; AMCL yaw is not recorded)
    odo_yaw = _yaw('odom_yaw')
    odo_yaw_err = np.full_like(t, np.nan)
    if valid_o.sum() > 2:
        odo_yaw_err = np.abs(wrap(odo_yaw - gt_yaw))
        odo_yaw_err[~valid_o] = np.nan

    # speed profile from gt
    speed = np.full_like(t, np.nan)
    if len(t) > 2:
        d = np.hypot(np.diff(gx), np.diff(gy))
        dt = np.diff(t)
        with np.errstate(divide='ignore', invalid='ignore'):
            speed[1:] = np.where(dt > 0, d / dt, np.nan)

    fig, axes = plt.subplots(2, 2, figsize=(13, 11))
    fig.suptitle('AMR validation metrics — %s' % args.csv.split('/')[-1],
                 fontsize=13)

    # 1. trajectory overlay
    ax = axes[0, 0]
    for (x0, y0, x1, y1) in WALLS:
        ax.add_patch(plt.Rectangle((x0, y0), x1 - x0, y1 - y0,
                                   color='0.55', zorder=1))
    for (x0, y0, x1, y1) in OBSTACLES:
        ax.add_patch(plt.Rectangle((x0, y0), x1 - x0, y1 - y0,
                                   color='0.75', zorder=1))
    ax.plot(gx, gy, 'g-', lw=1.6, label='ground truth')
    if valid_o.sum() > 2:
        i0 = np.argmax(valid_o)
        # Align the odom frame to the map frame with the full rigid
        # transform at the first valid sample: rotate by the initial yaw
        # offset, then translate to the matching ground-truth pose. The
        # ready-gate probe moves the robot slightly before recording, so
        # anchoring to the spawn marker would show a constant offset.
        dyaw0 = 0.0 if not np.isfinite(odo_yaw[i0]) else gt_yaw[i0] - odo_yaw[i0]
        c, s = math.cos(dyaw0), math.sin(dyaw0)
        rel_x = odo_x - odo_x[i0]
        rel_y = odo_y - odo_y[i0]
        ox = rel_x * c - rel_y * s + gx[i0]
        oy = rel_x * s + rel_y * c + gy[i0]
        ax.plot(ox, oy, 'b-', lw=1.2, alpha=0.8, label='odometry (aligned)')
    ax.plot(ax_, ay_, 'r--', lw=1.2, alpha=0.8, label='AMCL')
    ax.plot(*SPAWN, 'ko', ms=8, label='spawn')
    for name, (gx_, gy_) in GOALS.items():
        ax.plot(gx_, gy_, 'k+', ms=10, mew=2)
        ax.annotate(name, (gx_, gy_), textcoords='offset points',
                    xytext=(6, 6), fontsize=7)
    ax.set_xlim(-0.5, 10.5)
    ax.set_ylim(-0.5, 8.5)
    ax.set_aspect('equal')
    ax.set_title('Trajectory (map frame)')
    ax.legend(fontsize=8, loc='upper left')

    # 2. position errors
    ax = axes[0, 1]
    ax.plot(t, drift, 'b-', lw=1.2, label='odometry drift error')
    ax.plot(t, amcl_err, 'r-', lw=1.2, label='AMCL localization error')
    ax.set_xlabel('%s (s)' % tunit)
    ax.set_ylabel('error (m)')
    ax.set_title('Position error vs ground truth')
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)
    ymax = max(0.1, float(np.nanmax(np.concatenate([drift, amcl_err]))))
    ax.set_ylim(0, ymax * 1.15)

    # 3. yaw error
    ax = axes[1, 0]
    ax.plot(t, odo_yaw_err, 'b-', lw=1.2, label='odometry yaw error')
    ax.set_xlabel('%s (s)' % tunit)
    ax.set_ylabel('yaw error (rad)')
    ax.set_title('Heading error vs ground truth')
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)

    # 4. speed profile
    ax = axes[1, 1]
    ax.plot(t, speed, 'g-', lw=1.2)
    ax.set_xlabel('%s (s)' % tunit)
    ax.set_ylabel('speed (m/s)')
    ax.set_title('Ground-truth speed')
    ax.grid(alpha=0.3)

    fig.tight_layout(rect=(0, 0, 1, 0.97))
    fig.savefig(args.out, dpi=120)
    print('wrote %s' % args.out)
    return 0


if __name__ == '__main__':
    sys.exit(main())
