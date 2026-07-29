# amr_localization — Interface Spec

## Owned Topics

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `/scan` | `sensor_msgs/LaserScan` | Subscribe | LiDAR input for SLAM/AMCL |
| `/imu/data` | `sensor_msgs/Imu` | Subscribe | IMU input for EKF |
| `/odom` | `nav_msgs/Odometry` | Publish | EKF fused odometry (wheel + IMU) |
| `/tf` | `tf2_msgs/TFMessage` | Publish | `map → odom` from EKF/AMCL |
| `/map` | `nav_msgs/OccupancyGrid` | Publish | SLAM-generated or static map |
| `/amcl_pose` | `geometry_msgs/PoseWithCovarianceStamped` | Publish | AMCL localization estimate |

## Owned Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `base_frame_id` | string | `base_link` | Robot base frame |
| `odom_frame_id` | string | `odom` | Odometry frame |
| `world_frame` | string | `map` | World/map frame |
| `use_sim_time` | bool | `true` | Simulation clock flag |

## Owned Frames

| Frame | Parent | Child | Source |
|-------|--------|-------|--------|
| `odom → base_link` | EKF | `robot_localization` | EKF filter node |
| `map → odom` | SLAM/AMCL | `slam_toolbox` or `amcl` | Map->odom transform |

## Owned Actions

- `navigate_to_pose` (Nav2, consumed by amcl for initial pose)

## Owned Services

- `amcl/set_local_initial_pose`
- `slam_toolbox/save_map`
