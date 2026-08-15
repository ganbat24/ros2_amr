# amr_metrics — Interface Spec

Validation and metrics tooling. Unlike the other packages this one owns no
runtime nodes in `system.launch.py`; it is driven manually or from CI after a
stack is already up.

## Executables

| Executable | Purpose |
|------------|---------|
| `orchestrate` | Owns the stack: teardown, launch, readiness gate, campaigns. The only thing that starts or repairs a stack |
| `run_validation` | Scripted multi-goal tour. Measures only; fails loudly if the stack is not ready |
| `run_waypoints` | Continuous waypoint route with per-lap timing and AMCL drift |
| `tour_stats` | Aggregates a campaign of runs and checks provenance across them |
| `map_quality` | Scores a SLAM map against the simulated floorplan |
| `record_trajectory` | Records ground truth / odometry / AMCL pose to CSV |
| `ready_gate` | Probes the drive chain until odometry feedback proves it is live |
| `plot_metrics` | Renders a metrics report PNG from a recorded CSV |
| `scan_health` | LiDAR rate in both sim and wall time, plus the measured RTF |
| `motion_health` | Command rate vs achieved velocity — explains a stationary robot |
| `path_health` | Planned path characteristics |

All are installed as console scripts and run via `ros2 run amr_metrics <name>`.
(Before 2026-08-14 they did not install correctly — see `setup.cfg` history.)

Orchestration is deliberately separate from measurement: a harness that
restarts its own subject cannot produce a comparable baseline, and several
past results were confounded exactly that way.

## Consumed Topics

| Topic | Type | Used by |
|-------|------|---------|
| `/odom` | `nav_msgs/Odometry` | `ready_gate`, `record_trajectory` |
| `/amcl_pose` | `geometry_msgs/PoseWithCovarianceStamped` | `record_trajectory` |
| `/clock` | `rosgraph_msgs/Clock` | `record_trajectory` (sim-time stamping) |
| `/tf` | `tf2_msgs/TFMessage` | `record_trajectory` |

## Published Topics

| Topic | Type | Published by |
|-------|------|--------------|
| `/cmd_vel` | `geometry_msgs/Twist` | `ready_gate` (short probe only) |

## Actions

| Action | Type | Direction |
|--------|------|-----------|
| `/navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | Client (`run_validation`) |
| `/follow_waypoints` | `nav2_msgs/action/FollowWaypoints` | Client (`run_waypoints`) |

## Outputs

| Artifact | Description |
|----------|-------------|
| `traj.csv` | Per-sample ground truth, odometry and AMCL pose |
| `metrics_report.png` | Trajectory plot and error decomposition |
| `results.json` | Per-goal status and wall time, so campaigns can be aggregated |
| `environment.json` | Host, cores, middleware, Gazebo build, RTF, git SHA, whether the tree was dirty, and the launch arguments |
| `waypoint_results.json` | Lap times, missed waypoints, AMCL error per lap |
| `slam_map.yaml` / `.pgm` | Map saved from a `--use-slam` run |
| `map_quality.json` / `.png` | Map score against the floorplan, and the overlay |

`environment.json` records dirty trees and launch arguments because a bare
SHA is not provenance: the committed 4/4 result of 2026-08-14 was produced
from a working tree whose recorded commit did not contain the fix that made
it pass.
