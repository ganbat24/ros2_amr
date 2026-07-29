# amr_bringup — Test Matrix

## Build-Time Tests

None beyond lint.

## Runtime Tests

| Test | Expected Behavior |
|------|------------------|
| `system.launch.py` starts | All sub-launches start without error |
| Node graph | All expected nodes present in running graph |
| `/tf` exists | Full TF tree (`map → odom → base_link → child`) |

## Integration Tests

| Test | Expected Behavior |
|------|------------------|
| Full bringup | All Level B sensor topics appear within 30 s |
| `/cmd_vel` round-trip | Sending `/cmd_vel` reaches `diff_drive_controller` |
| EKF outputs `/odom` | At ≥ 30 Hz without frame jumps |

## Edge Cases

- Missing sub-package → launch fails with clear error
- World file not found → Gazebo fails to start
