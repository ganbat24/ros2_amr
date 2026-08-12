"""
Unit tests for the metrics plotter (amr_metrics.plot_metrics).

Covers the odometry-to-map alignment (the rigid transform at the first
valid sample) and the floorplan parser, which must stay in sync with the
generated amr_office world (no phantom obstacles, current wall layout).
"""
import math

import numpy as np

from amr_metrics.plot_metrics import align_odom, load_floorplan


def test_align_odom_recovers_rotated_translated_odom():
    """
    A truth path seen through a rotated + translated odom frame must
    come back exactly after alignment (rotation by the initial yaw
    offset, translation to the matching truth pose).
    """
    t = np.arange(50, dtype=float)
    gx, gy = 1.5 + 0.1 * t, 1.5 + 0.02 * t
    gt_yaw = np.zeros(50)
    theta0, tx, ty = 0.2, 0.4, -0.3
    c0, s0 = math.cos(-theta0), math.sin(-theta0)
    odx = (gx - tx) * c0 - (gy - ty) * s0
    ody = (gx - tx) * s0 + (gy - ty) * c0
    odo_yaw = np.full(50, -theta0)

    ax_, ay_ = align_odom(odx, ody, odo_yaw, gx, gy, gt_yaw)
    np.testing.assert_allclose(ax_, gx, atol=1e-9)
    np.testing.assert_allclose(ay_, gy, atol=1e-9)


def test_align_odom_nan_yaw_falls_back_to_translation():
    """
    If the anchor yaw is missing, fall back to a translation-only
    alignment anchored at the matching ground-truth pose.
    """
    t = np.arange(10, dtype=float)
    gx, gy = 0.1 * t, 0.0 * t
    gt_yaw = np.zeros(10)
    odo_yaw = np.full(10, np.nan)
    odx = gx + 0.7
    ody = gy - 0.2

    ax_, ay_ = align_odom(odx, ody, odo_yaw, gx, gy, gt_yaw)
    assert math.isclose(ax_[0], gx[0])
    assert math.isclose(ay_[0], gy[0])
    assert math.isclose(ax_[-1], gx[-1])
    assert math.isclose(ay_[-1], gy[-1])


def test_align_odom_empty_input_is_all_nan():
    """No valid odom samples -> all-NaN output (plotter skips it)."""
    n = np.full(10, np.nan)
    ax_, ay_ = align_odom(n, n, np.zeros(10), n, n, np.zeros(10))
    assert not np.isfinite(ax_).any()
    assert not np.isfinite(ay_).any()


def test_floorplan_matches_current_world():
    """
    The parsed floorplan must match the generated world: W2's gap into
    the top-right room ends at y 6.2, the O5 pinch is gone, and O1 sits
    at its current position.
    """
    walls, obstacles = load_floorplan()
    w2 = [r for r in walls if abs(r[0] - 7.7) < 0.01 and abs(r[2] - 7.9) < 0.01]
    assert w2, 'W2 wall not found'
    assert abs(w2[0][3] - 6.2) < 0.01, 'W2 end moved: %r' % w2[0]

    phantom = [
        r for r in obstacles
        if abs((r[0] + r[2]) / 2 - 5.5) < 0.01 and abs((r[1] + r[3]) / 2 - 5.5) < 0.01
    ]
    assert not phantom, 'phantom O5 obstacle still drawn: %r' % phantom

    assert any(
        abs(r[0] - 3.3) < 0.01 and abs(r[1] - 1.7) < 0.01
        and abs(r[2] - 3.9) < 0.01 and abs(r[3] - 2.3) < 0.01
        for r in obstacles
    ), 'O1 not at its current position: %r' % obstacles
