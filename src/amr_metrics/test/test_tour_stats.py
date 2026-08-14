"""
Unit tests for the campaign aggregator (amr_metrics.tour_stats).

Two behaviours matter enough to pin down. Wall-time statistics must ignore
failed goals, because a failure records the timeout budget and would drag
every median toward it — a campaign that got slower would look faster the
more it failed. And the provenance check must actually fire: it exists
because the committed 4/4 result of 2026-08-14 was produced from a dirty
tree, so its recorded SHA did not contain the fix that made it pass.
"""
import json
import os

from amr_metrics.tour_stats import load_runs, provenance, summarise


def _write_run(root, name, goals, sha='a' * 40, dirty=False, host='wsl2',
               rtf='1.0'):
    path = os.path.join(root, name)
    os.makedirs(path)
    with open(os.path.join(path, 'results.json'), 'w') as handle:
        json.dump({
            'goals': [{'goal': g, 'x': 0.0, 'y': 0.0, 'status': s,
                       'wall_s': w} for g, s, w in goals],
            'succeeded': sum(1 for _, s, _ in goals if s == 'SUCCEEDED'),
            'total': len(goals),
            'per_goal_timeout_s': 400.0,
        }, handle)
    with open(os.path.join(path, 'environment.json'), 'w') as handle:
        json.dump({'git_sha': sha, 'git_dirty': dirty, 'git_dirty_paths':
                   ['src/x.yaml'] if dirty else [], 'host': host,
                   'real_time_factor': rtf}, handle)
    return path


def test_wall_time_stats_ignore_failed_goals(tmp_path):
    """A goal that burned the full timeout must not enter the timing stats."""
    root = str(tmp_path)
    _write_run(root, 'run_01', [('g1', 'SUCCEEDED', 80.0)])
    _write_run(root, 'run_02', [('g1', 'ABORTED', 400.0)])
    _write_run(root, 'run_03', [('g1', 'SUCCEEDED', 90.0)])

    _, stats = summarise(load_runs([os.path.join(root, d)
                                    for d in sorted(os.listdir(root))]))
    assert stats['g1']['attempts'] == 3
    assert stats['g1']['succeeded'] == 2
    assert stats['g1']['walls'] == [80.0, 90.0]      # 400.0 excluded


def test_goal_order_follows_the_tour_not_the_alphabet(tmp_path):
    """Goals report in tour order; a dict-sorted table misreads the route."""
    root = str(tmp_path)
    _write_run(root, 'run_01', [('g3_bottom_right', 'SUCCEEDED', 10.0),
                                ('g1_top_right', 'SUCCEEDED', 20.0)])
    order, _ = summarise(load_runs([os.path.join(root, 'run_01')]))
    assert order == ['g3_bottom_right', 'g1_top_right']


def test_provenance_flags_a_dirty_run(tmp_path):
    """The exact failure this tool was written to catch."""
    root = str(tmp_path)
    _write_run(root, 'run_01', [('g1', 'SUCCEEDED', 80.0)])
    _write_run(root, 'run_02', [('g1', 'SUCCEEDED', 80.0)], dirty=True)

    clean, problems = provenance(load_runs(
        [os.path.join(root, d) for d in sorted(os.listdir(root))]))
    assert not clean
    assert any('uncommitted' in p for p in problems)


def test_provenance_flags_runs_spanning_commits(tmp_path):
    """Pooling runs from two different commits is not one campaign."""
    root = str(tmp_path)
    _write_run(root, 'run_01', [('g1', 'SUCCEEDED', 80.0)], sha='a' * 40)
    _write_run(root, 'run_02', [('g1', 'SUCCEEDED', 80.0)], sha='b' * 40)

    clean, problems = provenance(load_runs(
        [os.path.join(root, d) for d in sorted(os.listdir(root))]))
    assert not clean
    assert any('different commits' in p for p in problems)


def test_provenance_clean_campaign_reports_clean(tmp_path):
    root = str(tmp_path)
    _write_run(root, 'run_01', [('g1', 'SUCCEEDED', 80.0)])
    _write_run(root, 'run_02', [('g1', 'SUCCEEDED', 85.0)])

    clean, problems = provenance(load_runs(
        [os.path.join(root, d) for d in sorted(os.listdir(root))]))
    assert clean and problems == []


def test_runs_without_results_are_skipped_not_fatal(tmp_path):
    """A crashed run leaves a directory behind; the campaign still reports."""
    root = str(tmp_path)
    _write_run(root, 'run_01', [('g1', 'SUCCEEDED', 80.0)])
    os.makedirs(os.path.join(root, 'run_02'))          # crashed mid-tour

    runs = load_runs([os.path.join(root, d) for d in sorted(os.listdir(root))])
    assert len(runs) == 1
