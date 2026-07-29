# amr_description — Interface Spec

## Owned Topics

| Topic | Type | Direction | Frame | Rate |
|-------|------|-----------|-------|------|
| `/tf` | `tf2_msgs/TFMessage` | Publish | `map → odom → base_link → child` | ~50 Hz |
| `/joint_states` | `sensor_msgs/JointState` | Publish | — | ~50 Hz |
| `/tf_static` | `tf2_msgs/TFMessage` | Publish | Fixed transforms | ~0.1 Hz |

## Owned Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `robot_description` | string | path to URDF | xacro-processed URDF string |

## Owned Frames

| Frame | Parent | Child | Source |
|-------|--------|-------|--------|
| `base_link` | `odom` | wheel/imu/sensor links | robot_state_publisher (URDF) |
| `wheel_left_link` | `base_link` | — | robot_state_publisher |
| `wheel_right_link` | `base_link` | — | robot_state_publisher |
| `imu_link` | `base_link` | — | robot_state_publisher |
| `caster_front_link` | `base_link` | — | robot_state_publisher |
| `caster_rear_link` | `base_link` | — | robot_state_publisher |
| `laser_frame` | `base_link` | — | robot_state_publisher |
| `camera_link` | `base_link` | — | robot_state_publisher |
| `camera_optical_frame` | `camera_link` | — | robot_state_publisher |

## Owned Actions

None.

## Owned Services

None.
