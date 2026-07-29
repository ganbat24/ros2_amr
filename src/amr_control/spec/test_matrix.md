# amr_control — Test Matrix

## Build-Time Tests

| Test | Expected Behavior |
|------|------------------|
| YAML loads via `yaml.safe_load` | No parse errors, all keys valid YAML |
| `diff_drive_controller.yaml` schema | `left_wheel`, `right_wheel`, `wheel_separation`, `wheel_radius`, `enable_odom_tf` present and typed correctly |

## Runtime Tests

| Test | Expected Behavior |
|------|------------------|
| `controller_manager.launch.py` starts | `controller_manager` node present, no exception |
| `diff_drive_controller` loads | `controller_manager/list_controllers` shows `diff_drive_controller` in `active` state |
| `/cmd_vel` received | Controller processes and sets joint velocity commands |

## Integration Tests

| Test | Expected Behavior |
|------|------------------|
| `controller_manager.launch.py` + `gazebo_sim.launch.py` | `/joint_states` published at ≥ 10 Hz |
| `/cmd_vel` round-trip | Sending `geometry_msgs/Twist` on `/cmd_vel` produces wheel joint velocity changes |

## Edge Cases

- `enable_odom_tf: true` → controller publishes odom TF (forbidden by spec)
- Missing `left_wheel` parameter → controller fails to activate with clear error
