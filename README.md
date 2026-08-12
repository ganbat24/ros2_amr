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

# SLAM mapping mode (experimental: slam_toolbox 2.8.5 has an upstream
# params regression — AMCL is the default until upstream fixes it)
ros2 launch amr_bringup system.launch.py use_slam:=true

# Headless (server-only) Gazebo, no GUI
ros2 launch amr_bringup system.launch.py headless:=true
```

This launches, in order:
1. `robot_state_publisher` (no `joint_state_publisher` — Gazebo provides `/joint_states`)
2. Gazebo Harmonic (empty world) + robot spawn
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
| Navigation | Smac 2D + DWB | Smac offers better path quality than NavFn; DWB is the standard local planner |
| Odom source | EKF only | Controller does not publish odom TF — avoids conflict with EKF fused odometry |

## Contributing

See the contributing guidelines in the project documentation.

## Status

The stack is a **working simulation baseline**: the full pipeline
(control, sensors, localization, navigation) is verified live in Docker
on Jazzy + Gazebo Harmonic, with a 104-test suite (0 errors / 0
failures) run in CI. SLAM Toolbox mapping is selectable via
`use_slam:=true` but currently pinned to AMCL as the default due to an
upstream slam_toolbox 2.8.5 params regression. Real-hardware bringup
(`amr_control/controller_manager.launch.py`) is wired but untested —
it needs a hardware interface plugin in the URDF.

### Validation (amr_metrics)

`amr_metrics` scripts a goal tour and produces a metrics report
(trajectory overlay, odometry drift vs ground truth, AMCL localization
error, heading error, speed profile):

```bash
ros2 run amr_metrics run_validation \
  --goals g1_top_right,g2_top_left,g3_bottom_right,g4_home \
  --out-dir /tmp/amr_validation
```

Verified on the amr_office world (10 x 8 m, two doors): **g1 (top-right)
and g2 (top-left) succeed reliably** — multi-door crossings, 200+ s
sim-time runs, final pose within 0.25 m. Measured: odometry drift
< 0.35 m and AMCL localization error < 0.6 m over ~450 s wall
(~225 s sim), heading error < 0.06 rad. Known limits on the 2-vCPU
host: the gz_ros2_control/physics bridge needs ~60-90 s to warm up
after launch (the readiness gate waits for it), and the DWB's path
slop can still wedge the robot on long diagonal routes (g3/g4) —
corridors are tuned so g1/g2 are reliable.

## License

See [LICENSE](LICENSE).
