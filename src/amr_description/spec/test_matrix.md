# amr_description — Test Matrix

## Build-Time Tests

| Test | Expected Behavior |
|------|------------------|
| `xacro` processes `amr.urdf.xacro` | Output valid XML, no undefined macros |
| URDF contains all expected frames | `base_link`, `wheel_left_link`, `wheel_right_link`, `imu_link`, `laser_frame`, `camera_link`, `base_footprint` present in output |

## Runtime Tests

| Test | Expected Behavior |
|------|------------------|
| `rsp.launch.py` starts without error | `robot_state_publisher` and `joint_state_publisher` nodes present |
| `/joint_states` exists | Published within 5 s of launch |
| `/joint_states` rate | ≥ 1 Hz (default `joint_state_publisher` rate) |
| `/tf_static` exists with correct QoS | `transient_local` durability — received by subscriber using matching QoS |
| `/tf_static` contains expected frames | `base_link`, `camera_link`, `imu_link`, `laser_frame`, `laser_stand_link`, `caster_front_link`, `caster_rear_link` |
| TF topology | `base_footprint → base_link → child` structure consistent |

## Integration Tests

| Test | Expected Behavior |
|------|------------------|
| `rsp.launch.py` + `gazebo_sim.launch.py` (headless) | `/tf_static` contains expected frames, no frame lookup timeouts |

## Edge Cases

- URDF contains unknown macro → `xacro.process_file()` raises error
- QoS mismatch on `/tf_static` → subscriber with `volatile` QoS receives nothing (documented in interface spec)
