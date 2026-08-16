"""
Unit tests for the failed-goal log slicer (amr_metrics.goal_report).

Two things decide whether this tool is useful or misleading. The window must
contain only the failing goal's own lines — reading a whole tour's log is how
earlier sessions blamed the wrong component, because successful goals are
usually noisier than the failed one. And messages must group by kind, so that
one recurring failure reads as "440 x No valid trajectories" rather than 440
distinct one-off messages.
"""
from amr_metrics.goal_report import normalise, parse


LOG = """\
[controller_server-1] [INFO] [100.000000000] [controller_server]: Activating
[controller_server-1] [WARN] [150.000000000] [controller_server]: Failed to make progress
[controller_server-1] [WARN] [151.000000000] [controller_server]: No valid trajectories out of 440
[controller_server-1] [WARN] [152.000000000] [controller_server]: No valid trajectories out of 512
[bt_navigator-5] [ERROR] [400.000000000] [bt_navigator]: Timed out waiting
[controller_server-1] [WARN] [900.000000000] [controller_server]: Later goal noise
"""


def _entries(tmp_path):
    path = tmp_path / 'launch.log'
    path.write_text(LOG)
    return list(parse(str(path)))


def test_parses_level_stamp_node_and_message(tmp_path):
    entries = _entries(tmp_path)
    assert len(entries) == 6
    stamp, level, node, msg = entries[1]
    assert stamp == 150.0
    assert level == 'WARN'
    assert node == 'controller_server'
    assert msg == 'Failed to make progress'


def test_window_excludes_lines_from_other_goals(tmp_path):
    """The whole point: a goal is explained by its own window, not the tour."""
    entries = _entries(tmp_path)
    start, end = 140.0, 410.0
    window = [e for e in entries if start <= e[0] <= end
              and e[1] in ('WARN', 'ERROR', 'FATAL')]

    messages = [m for _, _, _, m in window]
    assert 'Later goal noise' not in messages     # after this goal ended
    assert 'Activating' not in messages           # INFO, and before it started
    assert len(window) == 4


def test_numbers_are_collapsed_so_one_failure_counts_as_one_kind():
    assert (normalise('No valid trajectories out of 440')
            == normalise('No valid trajectories out of 512'))


def test_normalise_keeps_distinct_messages_distinct():
    assert normalise('Failed to make progress') != normalise('No valid traj')


def test_unparseable_lines_are_skipped_not_fatal(tmp_path):
    path = tmp_path / 'launch.log'
    path.write_text('not a log line at all\n' + LOG)
    assert len(list(parse(str(path)))) == 6
