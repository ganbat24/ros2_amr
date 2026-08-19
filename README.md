# AMR Base Stack

[![CI](https://github.com/ganbat24/ros2_amr/actions/workflows/ci.yml/badge.svg)](https://github.com/ganbat24/ros2_amr/actions/workflows/ci.yml) [![ROS 2](https://img.shields.io/badge/ROS%202-Jazzy-green)](https://docs.ros.org/en/jazzy/) [![Ubuntu](https://img.shields.io/badge/Ubuntu-24.04-orange)](https://releases.ubuntu.com/noble/) [![Gazebo](https://img.shields.io/badge/Gazebo-Harmonic-blue)](https://gazebosim.org/docs/harmonic) [![C++20](https://img.shields.io/badge/C%2B%2B-20-00599C)](https://en.cppreference.com/w/cpp/20) [![Python](https://img.shields.io/badge/Python-3.12-3776AB)](https://www.python.org/) [![License](https://img.shields.io/badge/License-Apache%202.0-blue)](LICENSE)

Professional ROS 2 Jazzy AMR stack for differential-drive mobile robots — simulation-first on x86 host, extensible to real hardware and cloud runtime.

## Contents

- [Quick Start](#quick-start)
- [What This Stack Does](#what-this-stack-does)
- [Architecture](#architecture)
- [Package Overview](#package-overview)
- [Build & Development](#build--development)
- [Simulation Bringup](#simulation-bringup)
- [Waypoint Following](#waypoint-following)
- [Visualization](#visualization)
- [Controller Tuning](#controller-tuning)
- [TF Tree](#tf-tree)
- [Sensor Topics](#sensor-topics)
- [Decision Rationale](#decision-rationale)
- [Contributing](#contributing)
- [Status](#status)
- [License](#license)

## Quick Start

### Native (Linux + ROS 2 Jazzy)

```bash
git clone https://github.com/ganbat24/ros2_amr amr_ws
cd amr_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-up-to amr_bringup
source install/setup.bash
ros2 launch amr_bringup system.launch.py
```

### Docker

```bash
docker compose -f docker/docker-compose.yml up sim
```

### VS Code Dev Container

Open the repo in VS Code and let the dev container build automatically. It runs `colcon build --symlink-install` on creation.

## What This Stack Does

This stack provides a complete, simulation-validated baseline for a differential-drive autonomous mobile robot. It covers the full perception–localization–planning–control pipeline using ROS 2 Jazzy and Gazebo Harmonic, with all configuration files, launch scripts, and test scaffolding in place.

The design assumes **simulation-first** development: every package is tested in Gazebo before touching real hardware. Hardware integration (CAN, GPIO, real sensor drivers) is deferred until a specific platform is selected.

## Architecture

See [Architecture Overview](docs/architecture.md) for the system diagram, package map, and data flow details.

## Package Overview

| Package | Purpose |
|---------|---------|
| `amr_description` | URDF (xacro), ros2_control block, Gazebo sensor plugins, mesh placeholders |
| `amr_control` | `diff_drive_controller` config, `controller_manager` launch, velocity command routing |
| `amr_simulation` | Gazebo Harmonic world (SDF), robot spawn, `ros_gz_bridge` remappings |
| `amr_sensors` | Sensor bridge launch — LiDAR, IMU, camera via `ros_gz_bridge` + `image_proc` |
| `amr_localization` | EKF (`robot_localization`), SLAM Toolbox, AMCL, map server configs |
| `amr_navigation` | Nav2 parameter composition — Smac 2D global planner, DWB local planner, recovery behaviors |
| `amr_bringup` | Top-level `system.launch.py` that wires all sub-launches together |
| `amr_metrics` | Validation tours and campaigns, SLAM map scoring, health probes, plots |

## Build & Development

```bash
# Build all packages up to bringup
colcon build --packages-up-to amr_bringup
source install/setup.bash

# Run all tests
colcon test
colcon test-result

# Lint a specific package
colcon build --packages-select amr_control --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### Adding a New Package

1. Create `src/<package>/` with `package.xml`, `CMakeLists.txt`, `launch/`, `config/`, `spec/`, `test/`.
2. Write `spec/interface.md` (topics, services, parameters) — get sign-off.
3. Implement code. Write tests first, then code to pass them.
4. Run `ament_lint_auto` + `colcon test` — must be green before merge.

## Simulation Bringup

```bash
# Full stack: Gazebo + control + sensors + AMCL localization + Nav2 (default)
ros2 launch amr_bringup system.launch.py

# SLAM mapping mode (slam_toolbox; lifecycle-managed, verified mapping)
ros2 launch amr_bringup system.launch.py use_slam:=true

# Headless (server-only) Gazebo, no GUI
ros2 launch amr_bringup system.launch.py headless:=true

# Nav2 servers in a single component container instead of one process each
ros2 launch amr_bringup system.launch.py use_composition:=true
```

`use_composition:=true` loads the six Nav2 servers and their lifecycle
manager into one `component_container_isolated`. They become a single DDS
participant, which removes the discovery race that the process-per-node
path handles with a settle delay — the composed path orders the component
loads instead of waiting, with the lifecycle manager last. See
[Architecture](docs/architecture.md#process-layout) for why the container
must be the *isolated* variant.

This launches, in order:
1. `robot_state_publisher` (no `joint_state_publisher` — Gazebo provides `/joint_states`)
2. Gazebo Harmonic (`amr_office.sdf` by default) + robot spawn
3. `joint_state_broadcaster` + `diff_drive_controller` via spawners (the
   `gz_ros2_control` plugin creates the `controller_manager`)
4. Sensor bridges (LiDAR, IMU, camera) + `image_proc` rectify
5. EKF + AMCL + `map_server` (default) or EKF + SLAM Toolbox
   (`use_slam:=true`)
6. Nav2 lifecycle nodes

## Quick Start: Teleop

```bash
# Keyboard teleoperation (in a separate terminal)
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=false -r cmd_vel:=/cmd_vel
```

Plain `geometry_msgs/Twist` on `/cmd_vel` feeds `velocity_smoother`;
the smoothed output is bridged to `TwistStamped` on `/cmd_vel_stamped`
by `amr_navigation/twist_to_stamped.py`, which is what
`diff_drive_controller` (ros2_controllers 4.x, TwistStamped-only)
subscribes to.

```bash
# Or drive the robot directly (repeat at >2 Hz: cmd_vel_timeout is 0.5 s)
ros2 topic pub -r 5 /cmd_vel geometry_msgs/Twist "{linear: {x: 0.2}}"
```

## Waypoint Following

`waypoint_follower` drives a route continuously rather than one goal at a
time, dispatching a `navigate_to_pose` per waypoint. `number_of_loops`
repeats the route inside a single action call, which is how the long
autonomy run is driven:

```bash
# five passes of the four-goal route, with per-lap timing and drift
ros2 run amr_metrics orchestrate --tour --autonomy --loops 4 \
  --out-dir /tmp/amr_autonomy
```

`stop_on_failure` is false, so a missed waypoint is recorded in the result
and the route continues — how many of N it reached is the measurement, and
ending the run on the first miss would discard it.

## Visualization

Launch RViz2 with the navigation config:

```bash
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix amr_bringup --share)/rviz/amr_navigation.rviz --ros-args -p use_sim_time:=true
```

Or use the Docker compose RViz service:

```bash
docker compose -f docker/docker-compose.yml up rviz
```

## Controller Tuning

| Parameter | Value | What It Controls |
|-----------|-------|------------------|
| `wheel_separation` | 0.3 m | Track width — distance between left and right wheel centers. Must match URDF. |
| `wheel_radius` | 0.033 m | Wheel radius — affects distance-per-pulse and velocity scaling. Must match URDF. |
| `publish_rate` | 50.0 Hz | How often the controller publishes odometry and TF messages. |
| `enable_odom_tf` | false | Disabled because `robot_localization` EKF owns the `odom→base_link` TF. The controller still publishes `/odom` messages, but does not publish the TF. |

## TF Tree

```
map
  └── odom          ← robot_localization EKF
       └── base_link
            ├── wheel_left_link
            ├── wheel_right_link
            ├── imu_link
            ├── caster_front_link
            ├── caster_rear_link
            ├── laser_stand_link
            │    └── laser_frame
            │         └── laser_lens_link
            ├── camera_link
            │    ├── camera_lens_link
            │    └── camera_optical_frame
```

- `base_link → imu_link` is a static transform (IMU URDF joint).
- `base_link → laser_stand_link → laser_frame` and
  `base_link → camera_link` are static transforms.
- The EKF fuses `/odom` (wheel odometry from the controller) and `/imu/data` to produce the `odom→base_link` transform.
- `map→odom` is produced by SLAM Toolbox (mapping mode) or AMCL (localization mode).

## Sensor Topics

| Topic | Type | Rate | Source |
|-------|------|------|--------|
| `/scan` | `sensor_msgs/LaserScan` | 10 Hz | Gazebo ray sensor → `ros_gz_bridge` |
| `/imu/data` | `sensor_msgs/Imu` | 100 Hz | Gazebo IMU plugin → `ros_gz_bridge` |
| `/camera/image_raw` | `sensor_msgs/Image` | 30 Hz | Gazebo RGB camera → `ros_gz_bridge` → `image_proc` |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | 30 Hz | `ros_gz_bridge` (rectified by `image_proc` → `/image_proc/image_rect`) |

## Decision Rationale

| Decision | Choice | Why |
|----------|--------|-----|
| ROS 2 distro | Jazzy | LTS supported through 2029 |
| Simulator | Gazebo Harmonic | Official ROS 2 Gazebo integration, `ros_gz_bridge` for topic bridging |
| Robot morphology | Differential drive | Simplest non-holonomic platform for AMR research |
| Sensor suite | 2D LiDAR + IMU + RGB camera | Level B — sufficient for indoor navigation, extensible |
| Control | `ros2_control` + `diff_drive_controller` | Standard ROS 2 control framework, well-maintained |
| Localization | EKF + SLAM + AMCL | EKF fuses wheel + IMU; SLAM for mapping; AMCL for localization |
| Navigation | Smac 2D + RPP | Compared against DWB, MPPI, and Theta* (2026-08-19); untuned RPP beat a heavily-tuned DWB baseline by 2-3x on the hardest leg — see [Controller & Planner Comparison](#controller--planner-comparison) below. DWB retained as an alternate (`nav2_params_dwb.yaml`) |
| Odom source | EKF only | Controller does not publish odom TF — avoids conflict with EKF fused odometry |

## Contributing

See the contributing guidelines in the project documentation.

## Status

The stack is a **working simulation baseline**: the full pipeline
(control, sensors, localization, navigation) is verified live in Docker
on Jazzy + Gazebo Harmonic. `colcon test` over the eight `amr_*`
packages reports **114 test cases across 40 CTest targets, 0 errors and
0 failures** (measured 2026-08-14; the two counts differ because each
ament lint target expands into one case per file, and the workspace's
vendored `gz_ros2_control` overlay is excluded). SLAM Toolbox mapping is selectable via `use_slam:=true`; AMCL with the
pre-built map remains the default.

**Mapping is driven by `slam_survey`, not by the goal tour.** The tour
dispatches to fixed coordinates up to 8 m away, and slam_toolbox starts
with an empty map, so there is nothing to plan through — measured, it
scores 1/4 with three goals aborting within 3–6 s. `slam_survey` instead
drives a 21-waypoint coverage route by publishing `TwistStamped` directly
to `/cmd_vel_stamped`, bypassing nav2 entirely, and captures `/map` from
its own subscription:

```bash
ros2 run amr_metrics orchestrate --tour --survey --out-dir /tmp/slam_run
```

`map_quality` then scores the result against the world's own wall
geometry — the world is generated from rectangles, so ground truth is
exact and distances are computed analytically rather than against another
map. It is calibrated in both directions: 100% / 100% / 0.000 m on the
pre-built map, 0% coverage on an empty one.

Measured on a 21-waypoint survey (259 s, 21/21 waypoints,
`docs/validation/slam_map_quality.png`):

| metric | value |
|---|---|
| wall coverage | **93.1%** |
| occupied-cell precision | **100.0%** |
| occupied error median | **0.009 m** (p95 0.091 m, max 0.115 m) |
| explored fraction | **99.0%** |

1960 occupied cells against 2338 in the true geometry, none of them
spurious. The map is saved in **world coordinates**: slam_toolbox origins
its map at the robot's start pose, so the survey shifts the origin by the
measured start position. Without that shift the same map scores 19.8%
coverage at 0.618 m median error — the offset, not the map. Real-hardware
bringup (`amr_control/controller_manager.launch.py`) is wired but
untested — it needs a hardware interface plugin in the URDF.

### Validation (amr_metrics)

`amr_metrics` scripts a goal tour and produces a metrics report
(trajectory overlay, odometry drift vs ground truth, AMCL localization
error, heading error, speed profile):

```bash
# one command: clean teardown, launch, readiness gate, tour, report, teardown
ros2 run amr_metrics orchestrate --tour --out-dir /tmp/amr_validation

# or, against a stack you brought up yourself
ros2 run amr_metrics run_validation \
  --goals g1_top_right,g2_top_left,g3_bottom_right,g4_home \
  --out-dir /tmp/amr_validation
```

Verified on the amr_office world (10 x 8 m, two doors): the **full
four-goal tour succeeds 4/4** — g1 top-right, g2 top-left, g3
bottom-right, g4 home — every leg crossing at least one door (D1 and the
W2 gap into the top-right room), each final pose within 0.25 m.

Repeatable over a **ten-tour campaign: 40/40 goals, 10/10 tours, no
aborts and no retries** (2026-08-14, `9b48dd6`, 12-core WSL2, RTF 1.0).
Every run came from the same commit with a clean working tree, which the
harness checks and reports rather than assumes. This campaign used DWB +
SmacPlanner2D, the default at the time — as of 2026-08-19 the default is
RPP + SmacPlanner2D (faster on every leg; see
[Controller & Planner Comparison](#controller--planner-comparison) below),
and this table describes what is now `nav2_params_dwb.yaml`:

| goal | success | median | min–max |
|---|---|---|---|
| g1 top-right | 10/10 | 100 s | 76–148 s |
| g2 top-left | 10/10 | 62 s | 56–75 s |
| g3 bottom-right | 10/10 | 172 s | 135–231 s |
| g4 home | 10/10 | 74 s | 71–94 s |

Reproduce with:

```bash
ros2 run amr_metrics orchestrate --campaign 10 --out-dir /tmp/camp
ros2 run amr_metrics tour_stats /tmp/camp/run_* --markdown
```

The spread matters as much as the median: g3 varies by 96 s across
identical runs, so any tuning claim resting on a single tour is inside the
noise. This project has made that mistake and the campaign tooling exists
because of it.

Committed report `docs/validation/metrics_report_full_tour.png` is the
median run of that campaign by total tour time (422 s of 361–507 s), not
the best one. Measured over it: AMCL localization error median
**0.063 m** (p95 0.140 m, max 0.202 m, **0.0%** of 2273 samples above
0.3 m) over 37.7 m travelled. Plots are in sim time (from `/clock`); the
odometry trace is aligned to ground truth with the full rigid transform
at run start.

`environment.json` beside the report records the host, core count,
middleware, Gazebo build, real-time factor, git SHA, **whether the
working tree was dirty, and the launch arguments**. The last two exist
because the previous committed report did not have them: it was produced
from a tree carrying an uncommitted `default_server_timeout` fix, so the
SHA recorded next to it named a commit that did not contain the change
which made the tour pass. This one reports `git_dirty: false`.

The tour is gated on drive-chain readiness: the gz_ros2_control bridge
needs a warm-up after launch, and `orchestrate` probes odometry until it
responds before dispatching any goal. Bring-up reaches `active` in about
12 s.

Under software rendering (no GPU passthrough) the simulation runs at a
real-time factor of about 0.63, so a tour takes proportionally longer in
wall-clock time. Sensor rates are unaffected in simulation time — the
LiDAR delivers its full nominal 10 Hz — because every sim-time consumer
is slowed by the same factor. `ros2 run amr_metrics scan_health` reports
the rate in both time bases along with the measured real-time factor.

### Controller & Planner Comparison

The tuned DWB + SmacPlanner2D baseline (then the default) was A/B'd against
three untuned nav2 alternatives — Regulated Pure Pursuit, MPPI, Theta* — one
plugin swapped at a time. g3, the baseline's slowest and most variable leg
(172 s median, 135–231 s over ten tours), dropped to 53–91 s under every
alternative — bigger than the baseline's own run-to-run spread. **RPP won
outright** — fastest on every leg, comparable-or-better reliability, and
did it at plugin defaults with zero tuning:

| controller + planner | success | g3 median | notes |
|---|---|---|---|
| RPP + SmacPlanner2D | 31/32 (97%) | 53 s | **default as of 2026-08-19** |
| MPPI + SmacPlanner2D | 12/12 (100%) | 57 s | ~15-20% slower than RPP |
| DWB + Theta* | 12/12 (100%) | 91 s | controller matters more than planner |
| DWB + SmacPlanner2D (tuned) | 40/40 (100%) | 172 s | former default, kept as `nav2_params_dwb.yaml` |
| DWB + SmacPlanner2D (untuned) | 10/12 (83%) | 76-78 s\* | \*only on goals that didn't abort — see retrospective |

A follow-up tuning pass tried raising RPP's `rotate_to_heading_min_angle`
(0.785→1.2 rad) — no measurable improvement at N=5, not adopted. Pairing
RPP with Theta* instead of SmacPlanner2D also showed no measurable gain,
confirming the controller was the lever that mattered, not the planner.

A companion piece,
[`docs/validation/dwb_tuning_retrospective.md`](docs/validation/dwb_tuning_retrospective.md),
reviews DWB's tuning history against this data: the tuning bought
reliability (83%→100%), not competitiveness — even fully tuned, DWB stayed
2-3x slower than RPP's untuned defaults on the hardest leg. Full results,
predictions-vs-actual, and evidence-strength caveats (mixed N across arms,
one flaky goal, one undiagnosed bring-up hang) are in
[`docs/validation/method_comparison.md`](docs/validation/method_comparison.md).

## License

See [LICENSE](LICENSE).
