#!/usr/bin/env python3
"""Why is the robot not moving: no command, or command with no response.

When a tour fails with "Failed to make progress", the robot is stationary —
but stationary has two completely different causes and they need opposite
fixes:

  not commanded : the controller is not producing velocity commands. Its
                  inputs (costmap, plan, TF) are starved or it cannot find a
                  valid trajectory. Look upstream at the sensor pipeline and
                  the planner.
  not responding: commands are being published and the robot does not move.
                  The drive chain is broken or saturated — gz_ros2_control,
                  the controller manager, or physics.

Measured on the recorded tour of 2026-08-14, the robot was stationary 76-91%
of the time on *every* goal including the ones that passed, which is what
makes this distinction the next thing worth knowing.

Usage (needs a running stack, ideally mid-navigation):
    ros2 run amr_metrics motion_health --duration 60
"""
import argparse
import sys
import time


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--duration', type=float, default=60.0)
    ap.add_argument('--cmd-topic', default='/cmd_vel',
                    help='controller output (default: /cmd_vel)')
    ap.add_argument('--odom-topic', default='/odom')
    ap.add_argument('--moving-threshold', type=float, default=0.02,
                    help='m/s (or rad/s) below which we call it stationary')
    args = ap.parse_args()

    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry

    class MotionHealth(Node):
        def __init__(self):
            super().__init__('motion_health')
            self.set_parameters([
                rclpy.parameter.Parameter('use_sim_time',
                                          rclpy.Parameter.Type.BOOL, True)])
            self.cmds = []      # (wall_t, linear, angular)
            self.odoms = []     # (wall_t, linear, angular)
            self.create_subscription(Twist, args.cmd_topic, self._on_cmd, 10)
            self.create_subscription(Odometry, args.odom_topic,
                                     self._on_odom, 10)

        def _on_cmd(self, msg):
            self.cmds.append((time.time(), abs(msg.linear.x),
                              abs(msg.angular.z)))

        def _on_odom(self, msg):
            t = msg.twist.twist
            self.odoms.append((time.time(), abs(t.linear.x),
                               abs(t.angular.z)))

    rclpy.init()
    node = MotionHealth()
    print('watching %s and %s for %.0f s ...'
          % (args.cmd_topic, args.odom_topic, args.duration), flush=True)
    end = time.time() + args.duration
    while time.time() < end and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.2)

    thr = args.moving_threshold
    n_cmd, n_odom = len(node.cmds), len(node.odoms)
    if n_odom < 2:
        print('\nRESULT: %d odometry messages — the drive chain is not '
              'reporting at all.' % n_odom)
        node.destroy_node()
        rclpy.shutdown()
        return 1

    cmd_rate = (n_cmd / args.duration) if n_cmd else 0.0
    odom_rate = n_odom / args.duration
    cmd_moving = [c for c in node.cmds if c[1] > thr or c[2] > thr]
    odom_moving = [o for o in node.odoms if o[1] > thr or o[2] > thr]

    # Pair each odom sample with the most recent command within 0.5 s.
    commanded_but_still = 0
    paired = 0
    ci = 0
    for ot, ol, oa in node.odoms:
        while ci + 1 < n_cmd and node.cmds[ci + 1][0] <= ot:
            ci += 1
        if not n_cmd:
            break
        ct, cl, ca = node.cmds[ci]
        if ot - ct > 0.5:
            continue                      # no recent command; not comparable
        paired += 1
        if (cl > thr or ca > thr) and (ol <= thr and oa <= thr):
            commanded_but_still += 1

    # Split linear from angular. A robot spinning in place is "moving" by
    # any velocity test while making no progress at all — and progress is
    # what the progress checker measures.
    cmd_lin = [c for c in node.cmds if c[1] > thr]
    cmd_ang = [c for c in node.cmds if c[2] > thr]
    cmd_spin_only = [c for c in node.cmds if c[2] > thr and c[1] <= thr]
    odom_lin = [o for o in node.odoms if o[1] > thr]
    odom_spin_only = [o for o in node.odoms if o[2] > thr and o[1] <= thr]

    print('\n=== motion over %.0f s ===' % args.duration)
    print('  /cmd_vel messages   %d  (%.1f Hz)' % (n_cmd, cmd_rate))
    print('  /odom messages      %d  (%.1f Hz)' % (n_odom, odom_rate))
    print('  commands asking for motion   %d / %d  (%.0f%%)'
          % (len(cmd_moving), n_cmd, 100 * len(cmd_moving) / max(n_cmd, 1)))
    print('  odom reporting motion        %d / %d  (%.0f%%)'
          % (len(odom_moving), n_odom, 100 * len(odom_moving) / n_odom))
    print('  -- linear vs angular --')
    print('  commands with linear v       %d  (%.0f%%)'
          % (len(cmd_lin), 100 * len(cmd_lin) / max(n_cmd, 1)))
    print('  commands with angular v      %d  (%.0f%%)'
          % (len(cmd_ang), 100 * len(cmd_ang) / max(n_cmd, 1)))
    print('  commands SPIN ONLY           %d  (%.0f%%)'
          % (len(cmd_spin_only), 100 * len(cmd_spin_only) / max(n_cmd, 1)))
    print('  odom with linear v           %d  (%.0f%%)'
          % (len(odom_lin), 100 * len(odom_lin) / n_odom))
    print('  odom SPIN ONLY               %d  (%.0f%%)'
          % (len(odom_spin_only), 100 * len(odom_spin_only) / n_odom))
    if node.cmds:
        avg_lin = sum(c[1] for c in node.cmds) / n_cmd
        print('  mean commanded |linear|      %.3f m/s (max_vel_x is 0.5)'
              % avg_lin)
    if paired:
        print('  commanded but stationary     %d / %d paired  (%.0f%%)'
              % (commanded_but_still, paired,
                 100 * commanded_but_still / paired))

    print('\n=== verdict ===')
    frac_cmd_moving = len(cmd_moving) / max(n_cmd, 1)
    frac_odom_moving = len(odom_moving) / n_odom
    frac_disobeyed = commanded_but_still / paired if paired else 0.0

    if n_cmd == 0:
        print('  * NOT COMMANDED: the controller published nothing at all.')
        print('    It has no plan, no costmap, or no valid trajectory. The')
        print('    robot is idle by instruction, not by failure to move.')
    elif cmd_rate < 5.0:
        print('  * NOT COMMANDED (starved): only %.1f Hz of commands against '
              'a 20 Hz controller_frequency.' % cmd_rate)
        print('    The control loop is not completing its iterations — look')
        print('    upstream at costmap update rate and the scan pipeline')
        print('    (ros2 run amr_metrics scan_health).')
    elif frac_disobeyed > 0.3:
        print('  * NOT RESPONDING: %.0f%% of samples had a motion command '
              'with no measured motion.' % (100 * frac_disobeyed))
        print('    The drive chain is the problem, not navigation —')
        print('    gz_ros2_control, the controller manager, or physics.')
    elif frac_cmd_moving < 0.3:
        print('  * COMMANDED TO STOP: commands arrive at a healthy rate but '
              '%.0f%% of them ask for (near) zero velocity.'
              % (100 * (1 - frac_cmd_moving)))
        print('    The controller is choosing not to move — recovery')
        print('    behaviours, goal-reached, or no admissible trajectory.')
    elif len(odom_spin_only) / n_odom > 0.4:
        print('  * SPINNING IN PLACE: %.0f%% of odom samples show rotation '
              'with no linear velocity.' % (100 * len(odom_spin_only) / n_odom))
        print('    The drive chain is fine and commands are flowing — the')
        print('    robot is turning without committing to forward motion,')
        print('    which is why the progress checker fires while everything')
        print('    downstream looks healthy. Look at DWB critic balance and')
        print('    the rotate-to-goal / oscillation settings, not at the')
        print('    sensor pipeline.')
    else:
        print('  Commands and motion agree; the robot is moving when told '
              '(%.0f%% of odom samples, %.0f%% with linear velocity).'
              % (100 * frac_odom_moving, 100 * len(odom_lin) / n_odom))

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
