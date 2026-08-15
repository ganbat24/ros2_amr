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
| `waypoint_task_executor_plugin` | string | `"wait_at_waypoint"` | Per-waypoint task plugin. Note the key is `..._plugin`; `waypoint_task_executor` is not declared |

## Launch Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `use_composition` | `false` | Load the nav2 servers into one `component_container_isolated` instead of one process each |
| `lifecycle_settle` | `8.0` | Seconds after the last managed node starts before the lifecycle manager is created. Ignored when `use_composition` is true, which orders the loads instead of waiting |
| `params_file` | `config/nav2_params.yaml` | Nav2 parameter YAML |
| `bt_xml_filename` | nav2_bt_navigator's default tree | Behavior tree XML |

## Owned Frames

None owned.

## Owned Actions

- `navigate_to_pose` (Nav2 BT navigator)
- `follow_waypoints` (`nav2_msgs/action/FollowWaypoints`, waypoint_follower) —
  drives a route by dispatching one `navigate_to_pose` per waypoint.
  `number_of_loops` repeats the route within a single goal, which is how the
  long autonomy run is driven. `stop_on_failure` is false, so a missed
  waypoint is recorded in the result and the route continues.

## Owned Services

- `navigate_to_pose`
- `reinitialize_global_localization`
- `clear_entire_costmap`
