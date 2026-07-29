# amr_sensors — Test Matrix

## Build-Time Tests

None beyond lint.

## Runtime Tests

| Test | Expected Behavior |
|------|------------------|
| `sensors.launch.py` starts | All bridge nodes and image_proc present |
| `/imu/data` published | At ≥ 50 Hz, covariance values populated |
| `/scan` published | At ≥ 5 Hz, 360 samples, range 0.1–8.0 m |
| `/camera/image_raw` published | At ≥ 10 Hz, correct encoding (R8G8B8) |
| `/camera/camera_info` published | Matches image dimensions |

## Integration Tests

| Test | Expected Behavior |
|------|------------------|
| With Gazebo Harmonic running | All sensor topics bridged correctly, no message loss |

## Edge Cases

- Gazebo Harmonic not running → bridge nodes start but no data flows (expected)
- Wrong gz transport topic name → no messages bridged (forbidden by spec)
