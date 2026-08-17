"""
Unit tests for the direct-drive mapping survey (amr_metrics.slam_survey).

The steering is deliberately simple, which makes the two functions it rests on
worth pinning. An angle-wrap that returns the unwrapped error makes the robot
take the long way round — a 179 degree turn becomes 181 the other way, which
in a 1.6 m doorway is the difference between passing and clipping the frame.
A clamp that does not bound the negative side lets a reverse command run away.
"""
import math

import pytest

from amr_metrics.slam_survey import ARRIVE_TOL, MAX_ANG, MAX_LIN, clamp, wrap


def test_wrap_keeps_angles_in_plus_minus_pi():
    for angle in (0.0, 1.0, -1.0, math.pi - 0.01, -math.pi + 0.01):
        assert -math.pi <= wrap(angle) <= math.pi


def test_wrap_takes_the_short_way_round():
    """An error of 350 degrees is really -10, not a near-full rotation."""
    assert wrap(math.radians(350)) == pytest.approx(math.radians(-10))
    assert wrap(math.radians(-350)) == pytest.approx(math.radians(10))


def test_wrap_is_stable_across_multiple_turns():
    assert wrap(4 * math.pi + 0.3) == pytest.approx(0.3)


def test_clamp_bounds_both_directions():
    assert clamp(5.0, 0.8) == 0.8
    assert clamp(-5.0, 0.8) == -0.8
    assert clamp(0.2, 0.8) == 0.2


def test_speed_limits_are_gentle_enough_to_register_scans():
    """
    slam_toolbox matches scans against its graph; a fast spin produces scans
    it cannot register. These bounds are the reason the survey is slow, so a
    later 'speed it up' edit should have to change a test to do it.
    """
    assert MAX_LIN <= 0.5
    assert MAX_ANG <= 1.0


def test_arrival_tolerance_is_tighter_than_the_route_clearance():
    """
    Waypoints are placed with >=0.5 m clearance; arriving within a larger
    radius than that would let the robot call a point reached from inside a
    wall's inflation.
    """
    assert ARRIVE_TOL < 0.5
