# amr_metrics — Dependencies

## Upstream (Consumed)

| Source | Provides |
|--------|----------|
| `amr_localization` (AMCL/EKF) | `/odom`, `/amcl_pose`, `map -> odom -> base_link` TF |
| `amr_navigation` (Nav2) | `/navigate_to_pose` action server |
| `amr_simulation` (Gazebo) | `/clock`, ground-truth pose via `gz topic` |
| `amr_control` | the drive chain `ready_gate` probes |

## Package Dependencies

| Package | Why |
|---------|-----|
| `rclpy` | Node, action client, parameter handling |
| `nav_msgs`, `geometry_msgs`, `sensor_msgs` | Message types |
| `rosgraph_msgs` | `/clock` subscription |
| `tf2_msgs` | TF capture |
| `nav2_msgs` | `NavigateToPose` action |
| `python3-matplotlib` | Report rendering |

## Downstream (Consumers)

Nothing in the stack depends on this package at runtime. CI and the operator
consume its artifacts.

## Environment Requirements

A tour needs a fully active stack: `/amcl` and `/map_server` in `active`, the
drive chain warmed (odometry responding), and exactly one stack running —
overlapping stacks silently corrupt results.
