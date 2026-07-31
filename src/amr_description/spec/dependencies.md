# amr_description — Dependencies

## Upstream (Consumed)

| Topic | Type | Consumed By |
|-------|------|-------------|
| — | — | This package does not consume runtime topics. |

## Downstream (Produced)

| Topic | Type | QoS | Consumed By |
|-------|------|-----|-------------|
| `/tf` | `tf2_msgs/TFMessage` | reliable, volatile | robot_localization, amcl, Gazebo Harmonic |
| `/tf_static` | `tf2_msgs/TFMessage` | reliable, transient_local | All nodes needing static transforms |
| `/joint_states` | `sensor_msgs/JointState` | reliable, volatile | robot_state_publisher, diff_drive_controller |

## Parameters

| Parameter | Set By | Description |
|-----------|--------|-------------|
| `robot_description` | launch argument | Xacro-processed URDF XML string, passed to `robot_state_publisher` and `joint_state_publisher` |

## External Dependencies

- `xacro` (build-time)
- `urdfdom` (build-time)
- `robot_state_publisher` (runtime) — publishes `/tf`, `/tf_static`
- `joint_state_publisher` (runtime) — publishes `/joint_states` at 1 Hz
- `gz_ros2_control` (runtime, simulation only)
- `ros_gz_sim` (runtime, simulation only)
