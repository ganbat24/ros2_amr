"""
Unit tests for the waypoint autonomy run (amr_metrics.run_waypoints).

The point of this run is to catch degradation over time, so the two pieces
that decide whether degradation is visible are the ones worth pinning: lap
boundaries derived from feedback, and the per-lap error bucketing that turns
"median 0.06 m" into "0.04, 0.06, 0.14 and climbing".
"""
import time

from amr_metrics.run_waypoints import LapCounter, amcl_error_by_lap


def test_lap_counter_starts_at_one_lap():
    counter = LapCounter()
    for index in (0, 1, 2, 3):
        counter.update(index)
    assert len(counter.lap_times()) == 1


def test_lap_counter_marks_a_boundary_when_the_index_restarts():
    counter = LapCounter()
    for index in (0, 1, 2, 3, 0, 1, 2, 3, 0, 1):
        counter.update(index)
    assert len(counter.lap_times()) == 3


def test_lap_counter_ignores_a_repeated_index():
    """Feedback repeats while the robot works on one waypoint."""
    counter = LapCounter()
    for index in (0, 0, 1, 1, 1, 2):
        counter.update(index)
    assert len(counter.lap_times()) == 1


def test_amcl_error_is_bucketed_per_lap(tmp_path):
    csv = tmp_path / 'traj.csv'
    csv.write_text(
        'wall_t,sim_t,gt_x,gt_y,gt_yaw,odom_x,odom_y,odom_yaw,amcl_x,amcl_y\n'
        # lap 1: 0.1 m off
        '1.0,1.0,0.0,0.0,0,0,0,0,0.1,0.0\n'
        '2.0,2.0,0.0,0.0,0,0,0,0,0.1,0.0\n'
        # lap 2: 0.5 m off — a run that is getting worse
        '11.0,11.0,0.0,0.0,0,0,0,0,0.5,0.0\n'
        '12.0,12.0,0.0,0.0,0,0,0,0,0.5,0.0\n')

    start = time.time()
    per_lap = amcl_error_by_lap(str(csv), [start, start + 10.0], start)

    assert [entry['lap'] for entry in per_lap] == [1, 2]
    assert per_lap[0]['median_error_m'] == 0.1
    assert per_lap[1]['median_error_m'] == 0.5


def test_rows_without_an_amcl_fix_are_skipped(tmp_path):
    """AMCL publishes nothing before it converges; those rows are blank."""
    csv = tmp_path / 'traj.csv'
    csv.write_text(
        'wall_t,sim_t,gt_x,gt_y,gt_yaw,odom_x,odom_y,odom_yaw,amcl_x,amcl_y\n'
        '1.0,1.0,0.0,0.0,0,0,0,0,,\n'
        '2.0,2.0,0.0,0.0,0,0,0,0,0.2,0.0\n')

    start = time.time()
    per_lap = amcl_error_by_lap(str(csv), [start], start)

    assert per_lap[0]['samples'] == 1
    assert per_lap[0]['median_error_m'] == 0.2


def test_empty_trajectory_returns_no_laps_rather_than_crashing(tmp_path):
    csv = tmp_path / 'traj.csv'
    csv.write_text(
        'wall_t,sim_t,gt_x,gt_y,gt_yaw,odom_x,odom_y,odom_yaw,amcl_x,amcl_y\n')
    assert amcl_error_by_lap(str(csv), [time.time()], time.time()) == []
