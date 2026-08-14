#!/usr/bin/env python3
"""Aggregate a campaign of validation tours into one comparable summary.

Three tours from a single start pose is not a repeatability claim, and until
now the only record of a tour's outcome was its stdout — so multi-run evidence
could not be aggregated at all, and every "it passes" statement in this repo
rested on however many runs happened to be remembered. This reads the
`results.json` each tour now writes and reports the distribution.

It also checks provenance across the campaign. Runs are only comparable if
they ran the same code on the same host: if the git SHAs differ, or any run
had a dirty tree, the numbers are pooled from different stacks and the summary
says so rather than averaging them silently.

Usage:
  ros2 run amr_metrics tour_stats /tmp/campaign/run_* [--label baseline]
  ros2 run amr_metrics tour_stats /tmp/campaign/run_* --markdown
"""
import argparse
import json
import os
import statistics
import sys


def load_runs(paths):
    """Read (results, environment) for every directory that has a result."""
    runs = []
    for path in sorted(paths):
        rpath = os.path.join(path, 'results.json')
        if not os.path.exists(rpath):
            print('  (skipping %s — no results.json)' % path, file=sys.stderr)
            continue
        with open(rpath) as handle:
            results = json.load(handle)
        env = {}
        epath = os.path.join(path, 'environment.json')
        if os.path.exists(epath):
            with open(epath) as handle:
                env = json.load(handle)
        runs.append((os.path.basename(path.rstrip('/')), results, env))
    return runs


def provenance(runs):
    """Return (ok, list of human-readable problems) for the campaign."""
    problems = []
    shas = {env.get('git_sha', '')[:7] for _, _, env in runs if env}
    if len(shas) > 1:
        problems.append(
            'runs span %d different commits (%s) — these are not one campaign'
            % (len(shas), ', '.join(sorted(s for s in shas if s))))
    dirty = [name for name, _, env in runs if env.get('git_dirty')]
    if dirty:
        problems.append(
            '%d run(s) had uncommitted changes: %s — the recorded SHA does '
            'not describe the code that ran' % (len(dirty), ', '.join(dirty)))
    hosts = {env.get('host', '') for _, _, env in runs if env}
    if len(hosts) > 1:
        problems.append('runs span different hosts (%s)'
                        % ', '.join(sorted(hosts)))
    rtfs = {env.get('real_time_factor', '') for _, _, env in runs if env}
    if len(rtfs) > 1:
        problems.append('runs span different real-time factors (%s)'
                        % ', '.join(sorted(rtfs)))
    return (not problems), problems


def summarise(runs):
    """Per-goal success counts and wall-time stats over the campaign."""
    order, stats = [], {}
    for _, results, _ in runs:
        for entry in results['goals']:
            goal = entry['goal']
            if goal not in stats:
                order.append(goal)
                stats[goal] = {'attempts': 0, 'succeeded': 0, 'walls': []}
            slot = stats[goal]
            slot['attempts'] += 1
            if entry['status'] == 'SUCCEEDED':
                slot['succeeded'] += 1
                # Only successful goals carry a meaningful duration; a failure
                # records the timeout budget, which would skew every median.
                slot['walls'].append(entry['wall_s'])
    return order, stats


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('dirs', nargs='+', help='tour output directories')
    ap.add_argument('--label', default='campaign')
    ap.add_argument('--markdown', action='store_true',
                    help='emit a markdown table for the README')
    ap.add_argument('--json', dest='as_json', action='store_true')
    args = ap.parse_args()

    runs = load_runs(args.dirs)
    if not runs:
        print('no runs with results.json found', file=sys.stderr)
        return 1

    order, stats = summarise(runs)
    clean, problems = provenance(runs)
    full = sum(1 for _, r, _ in runs if r['succeeded'] == r['total'])
    goals_ok = sum(r['succeeded'] for _, r, _ in runs)
    goals_all = sum(r['total'] for _, r, _ in runs)

    if args.as_json:
        json.dump({
            'label': args.label,
            'runs': len(runs),
            'tours_fully_passed': full,
            'goals_succeeded': goals_ok,
            'goals_attempted': goals_all,
            'provenance_clean': clean,
            'provenance_problems': problems,
            'per_goal': {
                g: {
                    'attempts': stats[g]['attempts'],
                    'succeeded': stats[g]['succeeded'],
                    'median_s': (round(statistics.median(stats[g]['walls']), 1)
                                 if stats[g]['walls'] else None),
                    'min_s': min(stats[g]['walls']) if stats[g]['walls'] else None,
                    'max_s': max(stats[g]['walls']) if stats[g]['walls'] else None,
                } for g in order},
        }, sys.stdout, indent=2)
        print()
        return 0

    if args.markdown:
        print('| goal | success | median | min–max |')
        print('|---|---|---|---|')
        for g in order:
            s = stats[g]
            walls = s['walls']
            span = ('%.0f–%.0f s' % (min(walls), max(walls))) if walls else '—'
            med = ('%.0f s' % statistics.median(walls)) if walls else '—'
            print('| %s | %d/%d | %s | %s |'
                  % (g, s['succeeded'], s['attempts'], med, span))
        print()
        print('%d tours, %d/%d goals, %d/%d tours fully passed.'
              % (len(runs), goals_ok, goals_all, full, len(runs)))
        if not clean:
            print()
            for problem in problems:
                print('> provenance: %s' % problem)
        return 0

    print('== %s: %d tours ==' % (args.label, len(runs)))
    for name, results, _ in runs:
        marks = ''.join(
            '.' if e['status'] == 'SUCCEEDED' else 'x' for e in results['goals'])
        print('  %-20s %d/%d  %s'
              % (name, results['succeeded'], results['total'], marks))
    print()
    print('  %-16s %-9s %-9s %s' % ('goal', 'success', 'median', 'min-max'))
    for g in order:
        s = stats[g]
        walls = s['walls']
        print('  %-16s %d/%-7d %-9s %s' % (
            g, s['succeeded'], s['attempts'],
            ('%.0f s' % statistics.median(walls)) if walls else '-',
            ('%.0f-%.0f s' % (min(walls), max(walls))) if walls else '-'))
    print()
    print('  tours fully passed : %d/%d' % (full, len(runs)))
    print('  goals succeeded    : %d/%d (%.0f%%)'
          % (goals_ok, goals_all, 100.0 * goals_ok / goals_all))
    if clean:
        print('  provenance         : clean — one commit, no dirty trees')
    else:
        print('  provenance         : NOT CLEAN')
        for problem in problems:
            print('    ! %s' % problem)
    return 0


if __name__ == '__main__':
    sys.exit(main())
