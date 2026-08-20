# AMR Base Stack

[![CI](https://github.com/ganbat24/ros2_amr/actions/workflows/ci.yml/badge.svg)](https://github.com/ganbat24/ros2_amr/actions/workflows/ci.yml) [![ROS 2](https://img.shields.io/badge/ROS%202-Jazzy-green)](https://docs.ros.org/en/jazzy/) [![Ubuntu](https://img.shields.io/badge/Ubuntu-24.04-orange)](https://releases.ubuntu.com/noble/) [![Gazebo](https://img.shields.io/badge/Gazebo-Harmonic-blue)](https://gazebosim.org/docs/harmonic) [![C++20](https://img.shields.io/badge/C%2B%2B-20-00599C)](https://en.cppreference.com/w/cpp/20) [![Python](https://img.shields.io/badge/Python-3.12-3776AB)](https://www.python.org/) [![License](https://img.shields.io/badge/License-Apache%202.0-blue)](LICENSE)

ROS 2 Jazzy stack for a differential-drive AMR — simulation-first, portable to real hardware.

RViz demo, full 4-goal tour at 5x speed:

<video src="https://github.com/user-attachments/assets/412aacff-7f63-47be-813f-19b6ea95c37c" controls autoplay loop muted playsinline width="600">
  Your browser doesn't support inline video — see
  <a href="docs/media/rviz_navigation_demo.mp4">docs/media/rviz_navigation_demo.mp4</a>.
</video>

## Contents

[Quick Start](#quick-start) · [What This Stack Does](#what-this-stack-does) · [Architecture](#architecture) · [Package Overview](#package-overview) · [Build & Development](#build--development) · [Simulation Bringup](#simulation-bringup) · [Waypoint Following](#waypoint-following) · [Visualization](#visualization) · [Controller Tuning](#controller-tuning) · [TF Tree](#tf-tree) · [Sensor Topics](#sensor-topics) · [Decision Rationale](#decision-rationale) · [Contributing](#contributing) · [SLAM Mapping](#slam-mapping) · [Validation](#validation-amr_metrics) · [Controller & Planner Comparison](#controller--planner-comparison) · [License](#license)

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

Covers the full perception–localization–planning–control pipeline in ROS 2 Jazzy + Gazebo Harmonic, with config, launch files, and tests in place. Every package is tested in sim before touching hardware — hardware integration (CAN, GPIO, real sensors) is deferred until a platform is chosen.

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

`use_composition:=true` loads the Nav2 servers into one
`component_container_isolated` instead of one process each.
Process-per-node stays the default — see
[Architecture → Process Layout](docs/architecture.md#process-layout) for the
measured comparison and why.

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

See the demo video at the top of this README for what this config shows over
a full 4-goal tour.

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
| Navigation | Smac 2D + RPP | Untuned RPP beat a heavily-tuned DWB baseline 2-3x on the hardest leg — see [Controller & Planner Comparison](#controller--planner-comparison). DWB kept as an alternate (`nav2_params_dwb.yaml`) |
| Odom source | EKF only | Controller does not publish odom TF — avoids conflict with EKF fused odometry |

## Contributing

See the contributing guidelines in the project documentation.

## SLAM Mapping

SLAM Toolbox mapping is selectable via `use_slam:=true`; AMCL with the
pre-built map remains the default. Mapping is driven by a dedicated
21-waypoint survey, not the goal tour — the tour has nothing to plan
through on an empty map:

```bash
ros2 run amr_metrics orchestrate --tour --survey --out-dir /tmp/slam_run
```

Measured against the world's exact wall geometry: **93.1% wall coverage,
100.0% occupied-cell precision, 0.009 m median error**
(`docs/validation/slam/slam_map_quality.png`).

## Validation (amr_metrics)

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

Current default (RPP + SmacPlanner2D) on the amr_office world (10 x 8 m,
two doors): **full four-goal tour 4/4**, each leg crossing at least one
door, final pose within 0.25 m. Over a **three-tour campaign: 11/12 goals
(92%), 2/3 tours clean** (2026-08-20, `421db18`, 12-core WSL2, RTF 1.0):

| goal | success | median | min–max |
|---|---|---|---|
| g1 top-right | 3/3 | 59 s | 59–61 s |
| g2 top-left | 2/3 | 43 s | 41–44 s |
| g3 bottom-right | 3/3 | 64 s | 54–65 s |
| g4 home | 3/3 | 51 s | 48–53 s |

The one miss (run 3, g2) was a `bt_navigator`→`planner_server`
goal-acknowledgement timeout, not root-caused. N=3 is a regression screen,
not a settled reliability number. AMCL localization error on the
committed reference run (`docs/validation/full_tour/`): median **0.068 m**,
max 0.189 m over 33.2 m travelled.

For comparison, the retired tuned-DWB default was **40/40 goals, 10/10
tours** over a ten-tour campaign (2026-08-14, `9b48dd6`) — deeper N, but
2-3x slower on every leg (see below).

```bash
ros2 run amr_metrics orchestrate --campaign 3 --out-dir /tmp/camp
ros2 run amr_metrics tour_stats /tmp/camp/run_* --markdown
```

Bring-up reaches `active` in ~12 s. Under software rendering (no GPU
passthrough) real-time factor is ~0.63, so wall-clock time scales up
accordingly; sim-time sensor rates are unaffected.

## Controller & Planner Comparison

The tuned DWB + SmacPlanner2D baseline (then the default) was A/B'd against
three untuned Nav2 alternatives on g3, its slowest/most variable leg (172 s
median over ten tours). **RPP won outright** — fastest on every leg,
comparable-or-better reliability, zero tuning:

| controller + planner | success | g3 median | notes |
|---|---|---|---|
| RPP + SmacPlanner2D | 42/44 (95%) | 53 s | **default as of 2026-08-19** |
| MPPI + SmacPlanner2D | 12/12 (100%) | 57 s | ~15-20% slower than RPP |
| DWB + Theta* | 12/12 (100%) | 91 s | controller matters more than planner |
| DWB + SmacPlanner2D (tuned) | 40/40 (100%) | 172 s | former default, kept as `nav2_params_dwb.yaml` |
| DWB + SmacPlanner2D (untuned) | 10/12 (83%) | 76-78 s\* | \*only on goals that didn't abort |

DWB's tuning history bought reliability (83%→100%), not competitiveness —
even tuned, it stayed 2-3x slower than RPP's untuned defaults. A follow-up
RPP tuning pass and an RPP+Theta* pairing both showed no measurable gain
(N=5): the controller was the lever, not the planner. Evidence depth:
alternatives at N=3 (regression-screen) against the baseline's N=10 — the
2-3x gap clears run-to-run noise, finer differences don't.

## License

See [LICENSE](LICENSE).
