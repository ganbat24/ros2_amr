#!/usr/bin/env python3
"""Measure the laser-scan pipeline: throughput problem or stamping problem.

These need completely different fixes and look identical from the outside —
"AMCL drops scans, navigation fails near tight geometry". Answering it with a
measurement instead of a guess is the whole point of this tool.

  throughput : scans are not produced fast enough (rendering is the
               bottleneck). Fix by lowering sensor cost — sample count,
               update rate — or by getting a real GL driver.
  stamping   : scans arrive at rate but carry timestamps tf2 cannot use, so
               consumers drop them. Fix the clock/TF wiring; no amount of
               GPU helps.

Usage (needs a running stack):
    ros2 run amr_metrics scan_health --duration 30
    ros2 run amr_metrics scan_health --duration 30 --topic /scan_restamped
"""
import argparse
import statistics
import sys
import time


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--topic', default='/scan')
    ap.add_argument('--duration', type=float, default=30.0)
    # 10 Hz is this robot's gpu_lidar update_rate (amr_description's
    # gazebo.xacro, 360 samples). Override for a different sensor config.
    ap.add_argument('--expected-hz', type=float, default=10.0,
                    help='nominal sensor rate (default: the AMR lidar, 10 Hz)')
    args = ap.parse_args()

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan

    class ScanHealth(Node):
        def __init__(self):
            super().__init__('scan_health')
            self.set_parameters([
                rclpy.parameter.Parameter('use_sim_time',
                                          rclpy.Parameter.Type.BOOL, True)])
            self.arrivals = []      # wall time of each message
            self.ages = []          # clock_now - header.stamp, seconds
            self.stamps = []        # header stamps, seconds
            self.create_subscription(LaserScan, args.topic, self._on_scan,
                                     qos_profile_sensor_data)

        def _on_scan(self, msg):
            now = self.get_clock().now()
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.arrivals.append(time.time())
            self.stamps.append(stamp)
            self.ages.append(now.nanoseconds * 1e-9 - stamp)

    rclpy.init()
    node = ScanHealth()
    print('listening on %s for %.0f s ...' % (args.topic, args.duration),
          flush=True)
    end = time.time() + args.duration
    while time.time() < end and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.2)

    n = len(node.arrivals)
    if n < 2:
        print('\nRESULT: received %d scans on %s — the topic is effectively '
              'dead.' % (n, args.topic))
        print('That is not a tuning problem; find out why nothing is '
              'publishing (sensor plugin, bridge, physics).')
        node.destroy_node()
        rclpy.shutdown()
        return 1

    gaps = [b - a for a, b in zip(node.arrivals, node.arrivals[1:])]
    measured_hz = (n - 1) / (node.arrivals[-1] - node.arrivals[0])
    median_gap = statistics.median(gaps)
    worst_gap = max(gaps)
    median_age = statistics.median(node.ages)
    worst_age = max(node.ages)

    # Stamp monotonicity: repeated or backwards stamps make tf2 drop messages
    # regardless of how fast they arrive.
    backwards = sum(1 for a, b in zip(node.stamps, node.stamps[1:]) if b <= a)

    print('\n=== scan pipeline over %.0f s ===' % args.duration)
    print('  messages           %d' % n)
    print('  measured rate      %.2f Hz' % measured_hz)
    print('  gap median / worst %.3f s / %.3f s' % (median_gap, worst_gap))
    print('  age median / worst %.3f s / %.3f s  (clock now - header.stamp)'
          % (median_age, worst_age))
    print('  non-increasing stamps %d' % backwards)

    expected = args.expected_hz
    verdict = []
    if expected:
        ratio = measured_hz / expected
        print('  expected rate      %.2f Hz  (delivering %.0f%%)'
              % (expected, ratio * 100))
        if ratio < 0.7:
            verdict.append(
                'THROUGHPUT: delivering %.0f%% of the nominal rate. The '
                'sensor pipeline cannot keep up — reduce sensor cost (sample '
                'count, update rate) or get a working GL driver. Tuning the '
                'planner will not help.' % (ratio * 100))

    # A scan that is old by more than a couple of periods when it arrives is a
    # stamping/latency problem: consumers with a TF tolerance will drop it.
    if median_age > max(0.2, 3 * median_gap):
        verdict.append(
            'STAMPING/LATENCY: scans are %.2f s old on arrival (median), far '
            'more than one period. Consumers will drop them against the TF '
            'cache no matter how fast they arrive. Check clock wiring and '
            'restampers before touching navigation.' % median_age)

    if backwards:
        verdict.append(
            'STAMPING: %d non-increasing stamps. tf2 and message filters '
            'discard these outright.' % backwards)

    if worst_gap > 5 * median_gap:
        verdict.append(
            'BURSTINESS: worst gap %.2f s vs %.2f s median — the pipeline '
            'stalls periodically even if the average looks fine.'
            % (worst_gap, median_gap))

    print('\n=== verdict ===')
    if verdict:
        for item in verdict:
            print('  * ' + item)
    else:
        print('  Scan pipeline looks healthy: steady rate, fresh stamps,')
        print('  monotonic. If navigation still fails, the cause is')
        print('  downstream (costmap, planner, controller) — not the scans.')

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
