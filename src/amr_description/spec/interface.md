# amr_description — Interface Spec

## Owned Topics

| Topic | Type | Direction | Durability | Reliability | Rate | Notes |
|-------|------|-----------|------------|-------------|------|-------|
| `/tf` | `tf2_msgs/TFMessage` | Publish | volatile | reliable | on-change | Dynamic transforms (wheel joints) |
| `/tf_static` | `tf2_msgs/TFMessage` | Publish | **transient_local** | reliable | latched | Fixed transforms only — subscribers **must** use `transient_local` durability |
| `/joint_states` | `sensor_msgs/JointState` | Publish | volatile | reliable | 1 Hz | Default `joint_state_publisher` rate |

### QoS Notes

- `/tf_static` is published **once** with `transient_local` durability. Late-joining subscribers will receive the last message only if they request `transient_local`. Using default (`volatile`) QoS silently receives nothing.
- `/tf` is published on every joint state update. Standard `volatile` QoS is sufficient.
- `/joint_states` publishes at 1 Hz by default. To increase, set `publish_rate` parameter on `joint_state_publisher`.

## Owned Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `robot_description` | string | — | Xacro-processed URDF XML string (not a file path) |

## Owned Frames

The TF tree root is `base_footprint`, not `odom` or `map`. Those frames are provided by upstream localization nodes (e.g. `amcl`, `robot_localization`).

```
base_footprint
  └── base_link
        ├── wheel_left_link      (continuous)
        ├── wheel_right_link     (continuous)
        ├── caster_front_link    (fixed)
        ├── caster_rear_link     (fixed)
        ├── imu_link             (fixed)
        ├── laser_stand_link     (fixed)
        │     └── laser_frame    (fixed)
        │           └── laser_lens_link (fixed)
        └── camera_link          (fixed)
              ├── camera_lens_link (fixed)
              └── camera_optical_frame (fixed)
```

| Frame | Parent | Type | Source |
|-------|--------|------|--------|
| `base_footprint` | — (root) | fixed | URDF |
| `base_link` | `base_footprint` | fixed | robot_state_publisher |
| `wheel_left_link` | `base_link` | continuous | robot_state_publisher |
| `wheel_right_link` | `base_link` | continuous | robot_state_publisher |
| `caster_front_link` | `base_link` | fixed | robot_state_publisher |
| `caster_rear_link` | `base_link` | fixed | robot_state_publisher |
| `imu_link` | `base_link` | fixed | robot_state_publisher |
| `laser_stand_link` | `base_link` | fixed | robot_state_publisher |
| `laser_frame` | `laser_stand_link` | fixed | robot_state_publisher |
| `laser_lens_link` | `laser_frame` | fixed | robot_state_publisher |
| `camera_link` | `base_link` | fixed | robot_state_publisher |
| `camera_lens_link` | `camera_link` | fixed | robot_state_publisher |
| `camera_optical_frame` | `camera_link` | fixed | robot_state_publisher |

Fixed joints → published on `/tf_static`.
Continuous joints → published on `/tf` when joint states are received.

## Owned Actions

None.

## Owned Services

None.
