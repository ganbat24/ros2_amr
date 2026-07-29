# amr_control — Interface Spec

## Owned Topics

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `/cmd_vel` | `geometry_msgs/Twist` | Subscribe | Velocity command for differential drive |
| `/diff_drive_controller/cmd_vel` | `geometry_msgs/Twist` | Subscribe | Internal controller command topic |

## Owned Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `left_wheel` | string | `left_wheel_joint` | Left wheel joint name |
| `right_wheel` | string | `right_wheel_joint` | Right wheel joint name |
| `wheel_separation` | double | `0.3` | Wheel track width (m) |
| `wheel_radius` | double | `0.033` | Wheel radius (m) |
| `publish_rate` | double | `50.0` | Controller update rate (Hz) |
| `enable_odom_tf` | bool | `false` | Controller must NOT publish odom TF (EKF owns odom) |
| `odom_frame_id` | string | `odom` | Odom frame name |
| `base_frame_id` | string | `base_link` | Base frame name |
| `cmd_vel_topic` | string | `/cmd_vel` | Command velocity topic |

## Owned Actions

None.

## Owned Services

- `controller_manager/switch_controller`
- `controller_manager/load_controller`
- `controller_manager/unload_controller`
