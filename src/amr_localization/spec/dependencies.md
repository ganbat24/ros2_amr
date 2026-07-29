# amr_localization — Dependencies

## Upstream (Consumed)

| Topic | Type | Consumed By |
|-------|------|-------------|
| `/scan` | `sensor_msgs/LaserScan` | `ekf_filter_node`, `slam_toolbox`, `amcl` |
| `/imu/data` | `sensor_msgs/Imu` | `ekf_filter_node`, `slam_toolbox` |
| `/odom` (from controller) | `nav_msgs/Odometry` | `ekf_filter_node` |
| `/map` | `nav_msgs/OccupancyGrid` | `amcl` |

## Downstream (Produced)

| Topic | Type | Consumed By |
|-------|------|-------------|
| `/odom` | `nav_msgs/Odometry` | Nav2, controller (EKF owns odom) |
| `/tf` | `tf2_msgs/TFMessage` | All nodes needing TF |
| `/map` | `nav_msgs/OccupancyGrid` | Nav2 costmap, AMCL |
| `/amcl_pose` | `geometry_msgs/PoseWithCovarianceStamped` | Nav2, user code |

## External Dependencies

- `robot_localization`
- `slam_toolbox`
- `nav2_amcl`
- `nav2_map_server`
