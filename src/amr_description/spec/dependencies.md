# amr_description — Dependencies

## Upstream (Consumed)

| Topic | Type | Consumed By |
|-------|------|-------------|
| — | — | This package does not consume runtime topics. |

## Downstream (Produced)

| Topic | Type | Consumed By |
|-------|------|-------------|
| `/tf` | `tf2_msgs/TFMessage` | robot_localization, amcl, Gazebo Harmonic |
| `/tf_static` | `tf2_msgs/TFMessage` | All nodes needing static transforms |
| `/joint_states` | `sensor_msgs/JointState` | robot_state_publisher, diff_drive_controller |

## External Dependencies

- `xacro` (build-time)
- `urdfdom` (build-time)
- `robot_state_publisher` (runtime)
- `joint_state_publisher_gui` (runtime, manual override)
- `gz_ros2_control` (runtime, simulation only)
- `ros_gz_sim` (runtime, simulation only)
