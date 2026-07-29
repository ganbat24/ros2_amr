# amr_control — Dependencies

## Upstream (Consumed)

| Topic | Type | Consumed By |
|-------|------|-------------|
| `/cmd_vel` | `geometry_msgs/Twist` | `diff_drive_controller` |

| Service | Type | Consumed By |
|---------|------|-------------|
| `controller_manager/switch_controller` | `controller_manager_msgs/SwitchController` | `controller_manager` |

## Downstream (Produced)

| Topic | Type | Consumed By |
|-------|------|-------------|
| `/joint_states` | `sensor_msgs/JointState` | `robot_state_publisher`, TF tree |
| `/odom` | `nav_msgs/Odometry` | `robot_localization` EKF (not published when `enable_odom_tf: false`) |
| `/tf` | `tf2_msgs/TFMessage` | Not produced (TF disabled by `enable_odom_tf: false`) |

## External Dependencies

- `ros2_control`
- `ros2_controllers` (diff_drive_controller)
- `controller_manager`
- `gz_ros2_control` (simulation)
