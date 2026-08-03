# amr_navigation — Interface Spec

## Owned Topics

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `/scan` | `sensor_msgs/LaserScan` | Subscribe | Global/local costmap obstacle layer |
| `/amcl_pose` | `geometry_msgs/PoseWithCovarianceStamped` | Subscribe | Localization estimate |
| `/tf` | `tf2_msgs/TFMessage` | Subscribe | `map → odom`, `odom → base_link` |
| `/cmd_vel` | `geometry_msgs/Twist` | Publish | Velocity command to diff_drive_controller |
| `/map` | `nav_msgs/OccupancyGrid` | Publish | Occupancy grid map |
| `/global_costmap/costmap` | `nav_msgs/OccupancyGrid` | Publish | Global costmap |
| `/local_costmap/costmap` | `nav_msgs/OccupancyGrid` | Publish | Local costmap |

## Owned Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `controller_plugins` | string[] | `["FollowPath"]` | Controller plugin names |
| `planner_plugins` | string[] | `["GridBased"]` | Planner plugin names |
| `use_sim_time` | bool | `true` | Simulation clock flag |

## Owned Frames

None owned.

## Owned Actions

- `navigate_to_pose` (Nav2 BT navigator)

## Owned Services

- `navigate_to_pose`
- `reinitialize_global_localization`
- `clear_entire_costmap`
