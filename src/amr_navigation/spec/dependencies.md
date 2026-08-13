# amr_navigation — Dependencies

## Upstream (Consumed)

| Topic | Type | Consumed By |
|-------|------|-------------|
| `/scan` | `sensor_msgs/LaserScan` | Global/local costmap obstacle layer |
| `/amcl_pose` | `geometry_msgs/PoseWithCovarianceStamped` | BT Navigator |
| `/tf` | `tf2_msgs/TFMessage` | All Nav2 nodes |
| `/map` | `nav_msgs/OccupancyGrid` | Global costmap static layer |
| `/odom` | `nav_msgs/Odometry` | Controller, BT Navigator |

## Downstream (Produced)

| Topic | Type | Consumed By |
|-------|------|-------------|
| `/cmd_vel` | `geometry_msgs/Twist` | `diff_drive_controller` |
| `/global_costmap/costmap` | `nav_msgs/OccupancyGrid` | Smac Planner |
| `/local_costmap/costmap` | `nav_msgs/OccupancyGrid` | DWB Local Planner |
| `/amcl_pose` | `geometry_msgs/PoseWithCovarianceStamped` | User code |

## External Dependencies

- `nav2_bt_navigator`
- `nav2_controller`
- `nav2_planner`
- `nav2_behaviors`
- `nav2_costmap_2d`
- `nav2_lifecycle_manager`
- `nav2_smac_planner`
- `nav2_dwb_controller`
- `nav2_smoother`
- `nav2_velocity_smoother`
