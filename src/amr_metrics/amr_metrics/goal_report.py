#!/usr/bin/env python3
"""Explain a failed goal by isolating its own slice of the launch log.

This is the one diagnostic method that actually found causes in this stack,
turned into a tool instead of left as advice. For a goal that failed, take
only the WARN/ERROR lines between its dispatch and its failure and count them
by type:

  * a **lone warning** names the cause — the goal that finally took the tour
    to a repeatable 4/4 logged exactly one warning in its whole duration, and
    it was about action-server acknowledgement latency, not navigation;
  * a **flood** points elsewhere — 22 `Failed to make progress` meant the
    controller was choosing to crawl, which no amount of reading that message
    would have revealed.

Reading a whole tour's log instead is how several sessions concluded the wrong
thing: the log contains every goal's noise, and the successful goals are
usually noisier than the failed one.

Usage:
  ros2 run amr_metrics goal_report /tmp/camp/run_01
  ros2 run amr_metrics goal_report /tmp/camp/run_* --all-goals
"""
import argparse
import json
import os
import re
import sys
from collections import Counter

# [component-3] [WARN] [1786708120.032211743] [node]: message
LINE = re.compile(
    r'^\[?(?P<proc>[^\]]*)\]?\s*\[(?P<level>DEBUG|INFO|WARN|ERROR|FATAL)\]\s*'
    r'\[(?P<stamp>\d+\.\d+)\]\s*\[(?P<node>[^\]]+)\]:\s*(?P<msg>.*)$')

# Collapse the varying parts so the same failure counts as one kind, not N.
NUMERIC = re.compile(r'-?\d+\.?\d*')


def normalise(message):
    """Strip numbers so 'No valid trajectories out of 440' groups with 512."""
    return NUMERIC.sub('N', message).strip()


def parse(log_path):
    """Yield (stamp, level, node, message) for every parseable log line."""
    with open(log_path, errors='replace') as handle:
        for raw in handle:
            match = LINE.match(raw.strip())
            if match:
                yield (float(match.group('stamp')), match.group('level'),
                       match.group('node'), match.group('msg'))


def report_goal(entries, goal, verbose):
    start, end = goal['started_epoch'], goal['ended_epoch']
    window = [e for e in entries if start <= e[0] <= end
              and e[1] in ('WARN', 'ERROR', 'FATAL')]

    print('  %-16s %-9s %6.0f s   %d WARN/ERROR line(s) in its window'
          % (goal['goal'], goal['status'], goal['wall_s'], len(window)))
    if not window:
        print('      nothing logged — the failure is not visible in the log; '
              'look at motion_health and the trajectory instead')
        return

    counts = Counter((node, normalise(msg)) for _, _, node, msg in window)
    for (node, msg), n in counts.most_common(8 if not verbose else 100):
        print('      %4d x [%s] %s' % (n, node, msg[:110]))
    if len(counts) == 1 and sum(counts.values()) <= 3:
        print('      -> a lone warning in the whole window: this names the '
              'cause, it is not background noise')


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('run_dirs', nargs='+')
    ap.add_argument('--all-goals', action='store_true',
                    help='report successful goals too, for comparison')
    ap.add_argument('--verbose', action='store_true')
    args = ap.parse_args()

    seen_any = False
    for run_dir in sorted(args.run_dirs):
        results_path = os.path.join(run_dir, 'results.json')
        log_path = os.path.join(run_dir, 'launch.log')
        if not os.path.exists(results_path):
            continue
        with open(results_path) as handle:
            results = json.load(handle)
        goals = results['goals']
        failed = [g for g in goals if g['status'] != 'SUCCEEDED']
        if not failed and not args.all_goals:
            continue

        seen_any = True
        print('== %s: %d/%d ==' % (os.path.basename(run_dir.rstrip('/')),
                                   results['succeeded'], results['total']))
        if not os.path.exists(log_path):
            print('  no launch.log preserved for this run — nothing to '
                  'attribute the failure to. Runs recorded before the harness '
                  'started preserving it cannot be diagnosed after the fact.')
            continue
        if 'started_epoch' not in goals[0]:
            print('  results.json predates dispatch timestamps; cannot cut '
                  'per-goal windows out of the log.')
            continue

        entries = list(parse(log_path))
        for goal in (goals if args.all_goals else failed):
            report_goal(entries, goal, args.verbose)

    if not seen_any:
        print('no failed goals found in %d run(s)' % len(args.run_dirs))
    return 0


if __name__ == '__main__':
    sys.exit(main())
