# amr_metrics — Interface Spec

Validation and metrics tooling. Unlike the other packages this one owns no
runtime nodes in `system.launch.py`; it is driven manually or from CI after a
stack is already up.

## Executables

| Executable | Purpose |
|------------|---------|
| `run_validation` | Scripted multi-goal tour: readiness gate, goal dispatch, recording, report |
| `record_trajectory` | Records ground truth / odometry / AMCL pose to CSV |
| `ready_gate` | Probes the drive chain until odometry feedback proves it is live |
| `plot_metrics` | Renders a metrics report PNG from a recorded CSV |

All four are installed as console scripts and run via `ros2 run amr_metrics <name>`.
(Before 2026-08-14 they did not install correctly — see `setup.cfg` history.)

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

## Outputs

| Artifact | Description |
|----------|-------------|
| `traj.csv` | Per-sample ground truth, odometry and AMCL pose |
| `metrics_report.png` | Trajectory plot and error decomposition |
