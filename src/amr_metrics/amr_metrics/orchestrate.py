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


def capture_environment(out_dir):
    """Record what this run actually ran on.

    Past results are hard to compare because this was never captured: the
    Gazebo build, the middleware and the real-time factor have all differed
    between environments while the numbers were reported as if comparable.
    """
    def first_line(cmd):
        out = _sh(cmd, timeout=20)
        return (out.stdout.strip().splitlines() or [''])[0]

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
        'git_sha': first_line('git -C "$(dirname %s)" rev-parse HEAD 2>/dev/null'
                              % os.path.abspath(out_dir)),
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
    return env


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
    args = ap.parse_args()

    if not any([args.teardown, args.launch, args.wait_ready, args.tour]):
        ap.error('nothing to do — pass --tour, or one of '
                 '--teardown/--launch/--wait-ready')

    if args.teardown and not args.tour:
        print('== teardown ==')
        return 0 if teardown() else 1

    if args.launch and not args.tour:
        print('== launch ==')
        print('  log: %s' % launch(headless=args.headless == 'true'))
        if not args.wait_ready:
            return 0

    if args.wait_ready and not args.tour:
        print('== waiting for lifecycle active ==')
        elapsed = wait_lifecycle_active(timeout=args.launch_timeout)
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
        print('  log: %s' % launch(headless=args.headless == 'true'))
        print('== waiting for lifecycle active ==')
        elapsed = wait_lifecycle_active(timeout=args.launch_timeout)
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
        result = subprocess.run(
            [sys.executable, '-m', 'amr_metrics.run_validation',
             '--out-dir', args.out_dir])
        return result.returncode

    print('STACK NEVER BECAME READY after %d attempt(s)' % args.attempts)
    return 1


if __name__ == '__main__':
    sys.exit(main())
