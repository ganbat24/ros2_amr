"""
Unit tests for the SLAM map scorer (amr_metrics.map_quality).

The geometry is where this can go quietly wrong. An occupancy grid's row 0 is
the *top* of the map (maximum y), so getting the row-to-world mapping upside
down mirrors every error metric without ever raising — the score would still
look plausible and would be measuring the reflection of the map. The distance
function is the other half: it is what makes "how far is this cell from a real
wall" exact rather than resolution-limited.
"""
import math

import numpy as np
import pytest

from amr_metrics.map_quality import cell_centres, dilate, distance_to_rects


def test_distance_is_zero_inside_and_exact_outside():
    rects = [(0.0, 0.0, 2.0, 1.0)]
    x = np.array([[1.0, 3.0, 1.0, -1.0]])
    y = np.array([[0.5, 0.5, 2.0, 0.5]])

    d = distance_to_rects(x, y, rects)

    assert d[0, 0] == 0.0                     # inside
    assert d[0, 1] == pytest.approx(1.0)      # 1 m right of xmax
    assert d[0, 2] == pytest.approx(1.0)      # 1 m above ymax
    assert d[0, 3] == pytest.approx(1.0)      # 1 m left of xmin


def test_distance_uses_the_corner_for_diagonal_points():
    """Off a corner the distance is the hypotenuse, not the axis gap."""
    d = distance_to_rects(np.array([[5.0]]), np.array([[4.0]]),
                          [(0.0, 0.0, 2.0, 1.0)])
    assert d[0, 0] == pytest.approx(math.hypot(3.0, 3.0))


def test_distance_takes_the_nearest_of_several_rectangles():
    rects = [(0.0, 0.0, 1.0, 1.0), (10.0, 10.0, 11.0, 11.0)]
    d = distance_to_rects(np.array([[10.5]]), np.array([[9.0]]), rects)
    assert d[0, 0] == pytest.approx(1.0)


def test_no_geometry_yields_infinite_distance_not_zero():
    """An empty floorplan must not read as 'everything is a perfect hit'."""
    d = distance_to_rects(np.array([[0.0]]), np.array([[0.0]]), [])
    assert np.isinf(d[0, 0])


def test_row_zero_is_the_top_of_the_map():
    """
    The convention that silently mirrors the score if it is inverted.

    With origin (0, 0) and 0.1 m cells over 4 rows, the map spans y in
    [0, 0.4]; row 0 must be the 0.35 band, not the 0.05 one.
    """
    meta = {'resolution': 0.1, 'origin': [0.0, 0.0, 0.0]}
    x, y = cell_centres((4, 3), meta)

    assert y[0, 0] == pytest.approx(0.35)
    assert y[3, 0] == pytest.approx(0.05)
    assert x[0, 0] == pytest.approx(0.05)
    assert x[0, 2] == pytest.approx(0.25)


def test_origin_offset_shifts_world_coordinates():
    meta = {'resolution': 0.05, 'origin': [-0.5, -0.5, 0.0]}
    x, y = cell_centres((2, 2), meta)
    assert x[0, 0] == pytest.approx(-0.475)
    assert y[1, 0] == pytest.approx(-0.475)


def test_rotated_origin_is_refused_rather_than_mismeasured():
    """A yaw'd map would need resampling; scoring it anyway is silent error."""
    meta = {'resolution': 0.05, 'origin': [0.0, 0.0, 0.4]}
    with pytest.raises(ValueError, match='rotated map origin'):
        cell_centres((2, 2), meta)


def test_dilate_grows_a_single_cell_by_the_radius():
    mask = np.zeros((5, 5), dtype=bool)
    mask[2, 2] = True

    assert dilate(mask, 0).sum() == 1
    assert dilate(mask, 1).sum() == 9        # 3x3 block
    assert dilate(mask, 1)[1, 1]
    assert not dilate(mask, 1)[0, 0]
