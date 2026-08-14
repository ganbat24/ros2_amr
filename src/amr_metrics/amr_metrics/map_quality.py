#!/usr/bin/env python3
"""Score a SLAM-produced occupancy grid against the simulated floorplan.

`use_slam:=true` has been advertised as "selectable and verified" on the
strength of the map visibly growing while the robot drives. That is not a
measurement, and it was made while roughly a third of the slam_toolbox
parameter file was silently inert — so SLAM has never run with the settings
the config implied, and nothing has ever checked whether the map it produces
is any good.

Ground truth here is exact: the world is generated from rectangles, so this
compares against the wall geometry that was actually simulated rather than
against another map. Distances are computed analytically against those
rectangles — no rasterised distance transform, and no new dependency.

Reported:
  wall coverage    fraction of true wall surface with an occupied cell nearby
                   — misses mean SLAM failed to see a wall at all
  occupied precision
                   fraction of occupied cells that lie near a true wall
                   — misses mean spurious obstacles, the failure that makes a
                   map unusable for planning even when it looks right
  occupied error   distance from each occupied cell to the true geometry
  explored         fraction of the true free interior mapped as known-free

Usage:
  ros2 run amr_metrics map_quality --map /tmp/slam_map.yaml
  ros2 run amr_metrics map_quality --map /tmp/slam_map.yaml --out-dir /tmp/q
"""
import argparse
import json
import os
import sys

import numpy as np
import yaml


def load_occupancy(map_yaml):
    """Read a map_server YAML + image pair into (occ, free, known, meta).

    Returns three boolean grids indexed [row, col] plus the metadata dict.
    """
    with open(map_yaml) as handle:
        meta = yaml.safe_load(handle)
    image_path = meta['image']
    if not os.path.isabs(image_path):
        image_path = os.path.join(os.path.dirname(os.path.abspath(map_yaml)),
                                  image_path)
    from PIL import Image
    pixels = np.array(Image.open(image_path).convert('L')).astype(float)

    # map_server convention: with negate 0, occupancy = (255 - value) / 255.
    negate = int(meta.get('negate', 0))
    occupancy = pixels / 255.0 if negate else (255.0 - pixels) / 255.0
    occupied = occupancy > float(meta.get('occupied_thresh', 0.65))
    free = occupancy < float(meta.get('free_thresh', 0.196))
    known = occupied | free
    return occupied, free, known, meta


def cell_centres(shape, meta):
    """World (x, y) of every cell centre, as two [row, col] arrays.

    Image row 0 is the *top* of the map, i.e. maximum y — getting this
    upside down silently mirrors every error metric instead of failing.
    """
    rows, cols = shape
    res = float(meta['resolution'])
    ox, oy = float(meta['origin'][0]), float(meta['origin'][1])
    if abs(float(meta['origin'][2])) > 1e-6:
        raise ValueError('rotated map origin (yaw=%.3f) is not supported; '
                         'every metric here assumes an axis-aligned grid'
                         % float(meta['origin'][2]))
    j = np.arange(cols)[None, :]
    i = np.arange(rows)[:, None]
    x = ox + (j + 0.5) * res
    y = oy + (rows - i - 0.5) * res
    return np.broadcast_to(x, (rows, cols)), np.broadcast_to(y, (rows, cols))


def distance_to_rects(x, y, rects):
    """Shortest distance from each (x, y) to the nearest rectangle.

    Exact and vectorised: for an axis-aligned rectangle the distance is
    hypot(max(xmin - x, 0, x - xmax), max(ymin - y, 0, y - ymax)), which is
    0 inside. Beats rasterising the walls and running a distance transform,
    and cannot be thrown off by the map's own resolution.
    """
    if not rects:
        return np.full(x.shape, np.inf)
    best = np.full(x.shape, np.inf)
    for xmin, ymin, xmax, ymax in rects:
        dx = np.maximum.reduce([xmin - x, np.zeros_like(x), x - xmax])
        dy = np.maximum.reduce([ymin - y, np.zeros_like(y), y - ymax])
        best = np.minimum(best, np.hypot(dx, dy))
    return best


def dilate(mask, radius_cells):
    """True where any cell within a square radius is True (numpy only)."""
    if radius_cells <= 0:
        return mask
    out = mask.copy()
    for di in range(-radius_cells, radius_cells + 1):
        for dj in range(-radius_cells, radius_cells + 1):
            out |= np.roll(np.roll(mask, di, axis=0), dj, axis=1)
    return out


def score(map_yaml, tolerance=0.15):
    from amr_metrics.plot_metrics import WALLS, OBSTACLES

    occupied, free, known, meta = load_occupancy(map_yaml)
    x, y = cell_centres(occupied.shape, meta)
    res = float(meta['resolution'])
    solids = list(WALLS) + list(OBSTACLES)
    if not solids:
        raise ValueError('no floorplan geometry parsed from the world SDF — '
                         'refusing to report a score against nothing')

    dist = distance_to_rects(x, y, solids)
    truth_solid = dist <= 0.0

    # Only judge the area the world actually covers; a SLAM map is usually
    # larger than the room and the padding is neither wall nor free space.
    xs = [r[0] for r in solids] + [r[2] for r in solids]
    ys = [r[1] for r in solids] + [r[3] for r in solids]
    inside = ((x >= min(xs)) & (x <= max(xs)) &
              (y >= min(ys)) & (y <= max(ys)))

    tol_cells = int(round(tolerance / res))
    occupied_near_truth = occupied & inside & (dist <= tolerance)
    occupied_inside = occupied & inside
    truth_covered = truth_solid & inside & dilate(occupied, tol_cells)

    truth_free = inside & ~truth_solid
    errors = dist[occupied_inside]

    n_occ = int(occupied_inside.sum())
    n_truth = int((truth_solid & inside).sum())
    n_free = int(truth_free.sum())
    return {
        'map': os.path.abspath(map_yaml),
        'resolution_m': res,
        'tolerance_m': tolerance,
        'wall_coverage': (float(truth_covered.sum()) / n_truth
                          if n_truth else None),
        'occupied_precision': (float(occupied_near_truth.sum()) / n_occ
                               if n_occ else None),
        'occupied_error_median_m': (float(np.median(errors))
                                    if n_occ else None),
        'occupied_error_p95_m': (float(np.percentile(errors, 95))
                                 if n_occ else None),
        'occupied_error_max_m': float(errors.max()) if n_occ else None,
        'explored_fraction': (float((truth_free & known).sum()) / n_free
                              if n_free else None),
        'cells_occupied': n_occ,
        'cells_truth_solid': n_truth,
        'cells_truth_free': n_free,
    }, (occupied, known, x, y, solids, inside)


def plot(result, detail, out_png):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle

    occupied, known, x, y, solids, inside = detail
    fig, ax = plt.subplots(figsize=(9, 7))
    ax.scatter(x[known & ~occupied], y[known & ~occupied], s=1,
               c='#dfe6ee', label='SLAM free')
    ax.scatter(x[occupied], y[occupied], s=2, c='#c0392b',
               label='SLAM occupied')
    for i, (xmin, ymin, xmax, ymax) in enumerate(solids):
        ax.add_patch(Rectangle((xmin, ymin), xmax - xmin, ymax - ymin,
                               fill=False, edgecolor='#16609a', lw=1.4,
                               label='true geometry' if i == 0 else None))
    ax.set_aspect('equal')
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.set_title('SLAM map vs simulated floorplan\n'
                 'coverage %.1f%%  precision %.1f%%  median error %.3f m'
                 % (100 * (result['wall_coverage'] or 0),
                    100 * (result['occupied_precision'] or 0),
                    result['occupied_error_median_m'] or 0))
    ax.legend(loc='upper right', markerscale=6, fontsize=8)
    fig.tight_layout()
    fig.savefig(out_png, dpi=130)
    print('  wrote %s' % out_png)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--map', required=True, help='map_server YAML')
    ap.add_argument('--tolerance', type=float, default=0.15,
                    help='metres a cell may be from true geometry and still '
                         'count as agreeing (default 0.15, three cells at '
                         'the usual 0.05 resolution)')
    ap.add_argument('--out-dir')
    args = ap.parse_args()

    result, detail = score(args.map, args.tolerance)
    print('== map quality: %s ==' % result['map'])
    for key in ('wall_coverage', 'occupied_precision'):
        value = result[key]
        print('  %-24s %s' % (key, '%.1f%%' % (100 * value)
                              if value is not None else 'n/a'))
    for key in ('occupied_error_median_m', 'occupied_error_p95_m',
                'occupied_error_max_m'):
        value = result[key]
        print('  %-24s %s' % (key, '%.3f m' % value
                              if value is not None else 'n/a'))
    value = result['explored_fraction']
    print('  %-24s %s' % ('explored_fraction',
                          '%.1f%%' % (100 * value)
                          if value is not None else 'n/a'))

    if args.out_dir:
        os.makedirs(args.out_dir, exist_ok=True)
        with open(os.path.join(args.out_dir, 'map_quality.json'), 'w') as fh:
            json.dump(result, fh, indent=2)
        plot(result, detail, os.path.join(args.out_dir, 'map_quality.png'))
    return 0


if __name__ == '__main__':
    sys.exit(main())
