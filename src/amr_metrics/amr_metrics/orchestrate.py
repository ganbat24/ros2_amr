#!/usr/bin/env python3
"""Stack orchestration for validation runs: teardown, launch, readiness.

Split out of run_validation.py deliberately. A harness that restarts its own
subject cannot produce a clean baseline — several past results were confounded
because the measurement tool was also killing and relaunching the stack while
measuring it. Orchestration lives here; run_validation only measures, and
fails loudly when the stack is not ready instead of fixing it.

Typical use:

    # one command: clean slate, launch, wait, tour, report
    ros2 run amr_metrics orchestrate --tour --out-dir /tmp/run1

    # or drive the phases yourself
    ros2 run amr_metrics orchestrate --teardown
    ros2 run amr_metrics orchestrate --launch --wait-ready
    ros2 run amr_metrics run_validation --out-dir /tmp/run1
"""
import argparse
import json
import os
import platform
import shutil
import subprocess
import sys
import time

# Process names belonging to the stack. Nav2 nodes run as plain executables,
# so patterns like "ros2" or "gz sim" miss most of them and they survive as
# orphans — overlapping stacks are indistinguishable from a degraded stack in
# the metrics, which is the single most common cause of inexplicably bad runs.
STACK_PROCESS_NAMES = [
    'ros2 launch', 'gz sim', 'gz_ros2_control', 'robot_state_publisher',
    'spawner', 'static_transform', 'diff_drive', 'joint_state',
    'scan_restamp.py', 'tf_restamp.py', 'ekf_node', 'nav2_amcl', 'amcl',
    'map_server', 'controller_server', 'planner_server', 'smoother_server',
    'behavior_server', 'bt_navigator', 'velocity_smoother',
    'collision_monitor', 'waypoint_follower', 'lifecycle_manager',
    'parameter_bridge', 'twist_to_stamped.py', 'async_slam_toolbox_node',
    # With use_composition the six nav2 servers and their lifecycle manager
    # live inside this one process and none of the names above match it.
    # Missing it would leave a whole composed nav2 stack running after a
    # teardown that reported success — the overlapping-stack failure this
    # list exists to prevent, in a new disguise.
    'component_container',
]

# Never kill these, whatever else they match. Several of the names above are
# broad enough to appear in an ssh command line that is *running the teardown*
# — pattern-based teardown has twice killed the session driving it.
NEVER_KILL = ['sshd', 'ssh ', 'systemd', '/init', 'tmux', 'screen ']


def _sh(cmd, timeout=60):
    return subprocess.run(['bash', '-lc', cmd], capture_output=True,
                          text=True, timeout=timeout)


def running_stack_processes():
    """Return (pid, cmd) for every live stack process."""
    out = _sh('ps -eo pid,cmd --no-headers')
    found = []
    for line in out.stdout.splitlines():
        line = line.strip()
        if not line:
            continue
        pid, _, cmd = line.partition(' ')
        if not pid.isdigit() or int(pid) == os.getpid():
            continue
        # Skip our own orchestrator/validation processes.
        if 'amr_metrics' in cmd:
            continue
        if any(guard in cmd for guard in NEVER_KILL):
            continue
        if any(name in cmd for name in STACK_PROCESS_NAMES):
            found.append((int(pid), cmd.strip()))
    return found


def teardown(verbose=True):
    """Kill every stack process, then clear stale DDS shared memory.

    Kills by PID rather than by `pkill -f <pattern>`: a pattern broad enough
    to catch these names also matches the very shell running the teardown,
    which kills the caller instead of the stack.
    """
    for attempt in range(3):
        procs = running_stack_processes()
        if not procs:
            break
        if verbose:
            print('  teardown pass %d: killing %d process(es)'
                  % (attempt + 1, len(procs)), flush=True)
        for pid, _ in procs:
            try:
                os.kill(pid, 9)
            except (ProcessLookupError, PermissionError):
                pass
        time.sleep(2)

    # A SIGKILLed FastRTPS participant leaves shared-memory segments behind;
    # on the next launch a new participant (observed: map_server) fails to
    # bind its SHM port and never becomes discoverable at all.
    _sh('rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null')

    leftover = running_stack_processes()
    if leftover and verbose:
        print('  WARNING: %d process(es) survived teardown:' % len(leftover))
        for pid, cmd in leftover[:5]:
            print('    %d %s' % (pid, cmd[:90]))
    return not leftover


def launch(headless=True, extra_args='', log_path='/tmp/amr_launch.log'):
    """Start the full stack detached. Returns the log path."""
    install = os.environ.get('COLCON_PREFIX_PATH', '').split(os.pathsep)[0]
    if not install:
        sys.exit('COLCON_PREFIX_PATH is empty — source the workspace first')
    cmd = (
        'nohup bash -lc \'source /opt/ros/{distro}/setup.bash && '
        'source "{install}/setup.bash" && '
        'ros2 launch amr_bringup system.launch.py headless:={hl} {extra} '
        '> {log} 2>&1\' >/dev/null 2>&1 &'
    ).format(
        distro=os.environ.get('ROS_DISTRO', 'jazzy'), install=install,
        hl='true' if headless else 'false', extra=extra_args, log=log_path)
    _sh(cmd)
    return log_path


def wait_lifecycle_active(nodes=('/amcl', '/map_server'), timeout=180.0):
    """Block until every lifecycle node reports active. Returns seconds, or
    None on timeout.

    Queries each node's get_state service directly rather than shelling out to
    `ros2 lifecycle get`. The CLI needs its daemon and its own discovery pass,
    and in a non-interactive shell it returns empty output for nodes that are
    demonstrably active — a readiness check that reports failure while the
    launch log says "Managed nodes are active" is worse than no check, because
    it sends you debugging the wrong thing.
    """
    import rclpy
    from rclpy.node import Node as RclpyNode
    from lifecycle_msgs.srv import GetState

    started_here = not rclpy.ok()
    if started_here:
        rclpy.init()
    probe = RclpyNode('lifecycle_probe')
    clients = {
        n: probe.create_client(GetState, '%s/get_state' % n)
        for n in nodes
    }

    start = time.time()
    try:
        while time.time() - start < timeout:
            states = {}
            for name, client in clients.items():
                if not client.service_is_ready():
                    states[name] = 'no-service'
                    continue
                future = client.call_async(GetState.Request())
                rclpy.spin_until_future_complete(probe, future, timeout_sec=5.0)
                result = future.result()
                states[name] = (
                    result.current_state.label if result else 'no-reply')
            if all(s == 'active' for s in states.values()):
                return time.time() - start
            rclpy.spin_once(probe, timeout_sec=2.0)
            time.sleep(1.0)
        print('  last seen: %s' % states, flush=True)
        return None
    finally:
        probe.destroy_node()
        if started_here:
            rclpy.shutdown()


def wait_drive_ready(retries=5, retry_wait=30.0):
    """Probe the drive chain until odometry proves the feedback path is live.

    For ~60-90 s after launch the command path works while odom feedback is
    frozen; navigation started in that window drives blind.
    """
    import rclpy
    from amr_metrics.ready_gate import Gate

    started_here = not rclpy.ok()
    if started_here:
        rclpy.init()
    gate = Gate()
    try:
        for i in range(retries):
            moved = gate.probe()
            print('  drive probe %d: odom moved %.3f m' % (i + 1, moved),
                  flush=True)
            if moved > 0.03:
                return True
            if i < retries - 1:
                time.sleep(retry_wait)
        return False
    finally:
        gate.destroy_node()
        if started_here:
            rclpy.shutdown()


def _workspace_root():
    """Best guess at the colcon workspace root, for recording the git SHA.

    COLCON_PREFIX_PATH points at <workspace>/install; the workspace itself is
    its parent. Falls back to the cwd.
    """
    prefix = os.environ.get('COLCON_PREFIX_PATH', '').split(os.pathsep)[0]
    return os.path.dirname(prefix) if prefix else os.getcwd()


def capture_environment(out_dir):
    """Record what this run actually ran on.

    Past results are hard to compare because this was never captured: the
    Gazebo build, the middleware and the real-time factor have all differed
    between environments while the numbers were reported as if comparable.
    """
    def first_line(cmd):
        out = _sh(cmd, timeout=20)
        return (out.stdout.strip().splitlines() or [''])[0]

    # Tracked files only: build outputs and scratch dirs are untracked and
    # would mark every run dirty for no reason.
    dirty_paths = sorted(
        line[3:] for line in _sh(
            'git -C "%s" status --porcelain --untracked-files=no 2>/dev/null'
            % _workspace_root(), timeout=20).stdout.splitlines() if line.strip()
    )

    env = {
        'timestamp': time.strftime('%Y-%m-%dT%H:%M:%S%z'),
        'host': platform.node(),
        'cores': os.cpu_count(),
        'kernel': platform.release(),
        'is_wsl': 'microsoft' in platform.release().lower(),
        'in_container': os.path.exists('/.dockerenv'),
        'ros_distro': os.environ.get('ROS_DISTRO', ''),
        'rmw': os.environ.get('RMW_IMPLEMENTATION', '<default>'),
        'ros_domain_id': os.environ.get('ROS_DOMAIN_ID', '0'),
        # The Gazebo that matters is the one visible with ROS sourced: the
        # vendored build shadows any apt install.
        'gz_version': first_line('gz sim --versions 2>/dev/null'),
        'gz_binary': shutil.which('gz') or '',
        'libgl_always_software': os.environ.get('LIBGL_ALWAYS_SOFTWARE', ''),
        # The workspace, not the output dir — out_dir is usually under /tmp,
        # which is not a git repo, and silently yielded an empty SHA.
        'git_sha': first_line(
            'git -C "%s" rev-parse HEAD 2>/dev/null' % _workspace_root()),
        # A SHA alone is not provenance. The 4/4 tours of 2026-08-14 were run
        # with the default_server_timeout fix still uncommitted, so the SHA
        # recorded beside them (6b0432f) does not contain the change that
        # produced the result. Record whether the tree was dirty, and which
        # tracked files differed, so an artifact always describes the code
        # that actually ran rather than the last commit that happened to
        # precede it.
        'git_dirty': bool(dirty_paths),
        'git_dirty_paths': dirty_paths,
        'real_time_factor': first_line(
            "grep -o '<real_time_factor>[^<]*' "
            "$(ros2 pkg prefix --share amr_simulation 2>/dev/null)"
            "/worlds/amr_office.sdf 2>/dev/null | head -1 | cut -d'>' -f2"),
        'restampers_running': [
            cmd for _, cmd in running_stack_processes()
            if 'restamp' in cmd
        ],
    }
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'environment.json')
    with open(path, 'w') as handle:
        json.dump(env, handle, indent=2)
    print('  environment recorded -> %s' % path, flush=True)
    for key in ('host', 'cores', 'rmw', 'gz_version', 'real_time_factor'):
        print('    %-18s %s' % (key, env[key]), flush=True)
    print('    %-18s %s' % ('git', env['git_sha'][:7] + (
        ' DIRTY (%d tracked file%s modified)' % (
            len(dirty_paths), '' if len(dirty_paths) == 1 else 's')
        if dirty_paths else ' clean')), flush=True)
    if dirty_paths:
        print('      this run does NOT correspond to a committed state:',
              flush=True)
        for rel in dirty_paths:
            print('        %s' % rel, flush=True)
    return env


def save_and_score_map(out_dir):
    """Save the live SLAM map and score it against the simulated floorplan.

    Must run while the stack is still up — map_saver_cli subscribes to /map,
    so once the stack is torn down there is nothing left to save. This is why
    it sits inside the tour block rather than after teardown.
    """
    stem = os.path.join(out_dir, 'slam_map')
    print('== saving SLAM map ==', flush=True)
    saved = _sh('ros2 run nav2_map_server map_saver_cli -f "%s" '
                '--ros-args -p save_map_timeout:=60.0' % stem, timeout=120)
    if not os.path.exists(stem + '.yaml'):
        print('  map_saver_cli produced no map; skipping scoring', flush=True)
        print('  %s' % (saved.stderr.strip().splitlines() or [''])[-1],
              flush=True)
        return False
    print('== scoring map against the simulated floorplan ==', flush=True)
    subprocess.run([sys.executable, '-m', 'amr_metrics.map_quality',
                    '--map', stem + '.yaml', '--out-dir', out_dir])
    return True


def run_campaign(args):
    """Run N tours back to back, each on a fresh stack, then summarise.

    Each tour is a separate process so one crashed run cannot take the
    campaign with it, and so every run gets the same teardown/launch/gate
    path as a solo `--tour` — a campaign whose runs differ from the runs it
    is meant to characterise is worth nothing.
    """
    os.makedirs(args.out_dir, exist_ok=True)
    with open(os.path.join(args.out_dir, 'campaign.json'), 'w') as handle:
        json.dump({'requested_runs': args.campaign,
                   'launch_args': args.launch_args,
                   'started': time.strftime('%Y-%m-%dT%H:%M:%S%z')},
                  handle, indent=2)

    run_dirs = []
    for i in range(1, args.campaign + 1):
        run_dir = os.path.join(args.out_dir, 'run_%02d' % i)
        print('\n########## campaign run %d/%d -> %s ##########'
              % (i, args.campaign, run_dir), flush=True)
        cmd = [sys.executable, '-m', 'amr_metrics.orchestrate', '--tour',
               '--out-dir', run_dir, '--headless', args.headless,
               '--attempts', str(args.attempts)]
        if args.launch_args:
            cmd += ['--launch-args', args.launch_args]
        try:
            subprocess.run(cmd, timeout=3600)
        except subprocess.TimeoutExpired:
            print('  run %d exceeded its hour budget; tearing down' % i,
                  flush=True)
            teardown()
        run_dirs.append(run_dir)

    print('\n########## campaign summary ##########', flush=True)
    subprocess.run([sys.executable, '-m', 'amr_metrics.tour_stats',
                    '--label', os.path.basename(args.out_dir.rstrip('/'))]
                   + run_dirs)
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--teardown', action='store_true',
                    help='kill every stack process and clear stale DDS shm')
    ap.add_argument('--launch', action='store_true', help='start the stack')
    ap.add_argument('--wait-ready', action='store_true',
                    help='wait for lifecycle active + a live drive chain')
    ap.add_argument('--tour', action='store_true',
                    help='teardown, launch, wait, then run the validation tour')
    ap.add_argument('--out-dir', default='/tmp/amr_validation')
    ap.add_argument('--headless', default='true', choices=['true', 'false'])
    ap.add_argument('--attempts', type=int, default=2,
                    help='relaunch attempts if the stack never becomes ready')
    ap.add_argument('--launch-timeout', type=float, default=240.0)
    ap.add_argument('--keep-alive', action='store_true',
                    help='leave the stack running after --tour (default: '
                         'always tear down, so a run cannot pin the machine)')
    ap.add_argument('--launch-args', default='',
                    help='extra launch arguments, e.g. "use_tf_restamp:=false"'
                         ' — the only supported way to A/B a launch option, '
                         'since ros2 param set does not reach plugins that '
                         'capture their config at construction')
    ap.add_argument('--campaign', type=int, metavar='N',
                    help='run N tours back to back into <out-dir>/run_NN, '
                         'each on a freshly launched stack, then summarise')
    ap.add_argument('--use-slam', action='store_true',
                    help='map with slam_toolbox instead of localising with '
                         'AMCL; after the tour, save the map and score it '
                         'against the simulated floorplan')
    args = ap.parse_args()

    # SLAM and AMCL are mutually exclusive and start different nodes, so the
    # readiness gate has to watch different ones. Waiting on /amcl in SLAM
    # mode times out against a stack that is perfectly healthy.
    if args.use_slam:
        args.launch_args = (args.launch_args + ' use_slam:=true').strip()
        lifecycle_nodes = ('/slam_toolbox',)
    else:
        lifecycle_nodes = ('/amcl', '/map_server')

    if not any([args.teardown, args.launch, args.wait_ready, args.tour,
                args.campaign]):
        ap.error('nothing to do — pass --tour or --campaign N, or one of '
                 '--teardown/--launch/--wait-ready')

    if args.campaign:
        return run_campaign(args)

    if args.teardown and not args.tour:
        print('== teardown ==')
        return 0 if teardown() else 1

    if args.launch and not args.tour:
        print('== launch ==')
        print('  log: %s' % launch(headless=args.headless == 'true',
                                   extra_args=args.launch_args))
        if not args.wait_ready:
            return 0

    if args.wait_ready and not args.tour:
        print('== waiting for lifecycle active ==')
        elapsed = wait_lifecycle_active(nodes=lifecycle_nodes,
                                        timeout=args.launch_timeout)
        if elapsed is None:
            print('FAILED: lifecycle nodes never reached active')
            return 1
        print('  active after %.0f s' % elapsed)
        return 0 if wait_drive_ready() else 1

    # --tour: the full sequence, with relaunch as an orchestration concern.
    for attempt in range(1, args.attempts + 1):
        print('== attempt %d/%d: teardown ==' % (attempt, args.attempts))
        teardown()
        print('== launch ==')
        print('  log: %s' % launch(headless=args.headless == 'true',
                                   extra_args=args.launch_args))
        print('== waiting for lifecycle active ==')
        elapsed = wait_lifecycle_active(nodes=lifecycle_nodes,
                                        timeout=args.launch_timeout)
        if elapsed is None:
            print('  lifecycle never reached active; retrying')
            continue
        print('  active after %.0f s' % elapsed)
        print('== drive-chain readiness ==')
        if not wait_drive_ready():
            print('  drive chain never became ready; retrying')
            continue

        print('== environment ==')
        capture_environment(args.out_dir)
        print('== validation tour ==')
        try:
            result = subprocess.run(
                [sys.executable, '-m', 'amr_metrics.run_validation',
                 '--out-dir', args.out_dir])
            if args.use_slam:
                save_and_score_map(args.out_dir)
            return result.returncode
        finally:
            # Always tear the stack down. A tour that leaves 25 processes and
            # a saturated machine behind is a bug in the harness, not a
            # detail — this was left running for hours once, pinning a
            # 12-core laptop at load 12. --keep-alive opts out when you
            # genuinely want to inspect the live stack afterwards.
            if not args.keep_alive:
                print('== teardown (post-tour) ==')
                teardown()
            else:
                print('== leaving the stack up (--keep-alive) ==')
                print('   tear down with: '
                      'ros2 run amr_metrics orchestrate --teardown')

    print('STACK NEVER BECAME READY after %d attempt(s)' % args.attempts)
    teardown()
    return 1


if __name__ == '__main__':
    sys.exit(main())
