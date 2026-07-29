# amr_simulation — Test Matrix

## Build-Time Tests

| Test | Expected Behavior |
|------|------------------|
| SDF world loads via `gz sim` | No parse errors, physics engine initializes |

## Runtime Tests

| Test | Expected Behavior |
|------|------------------|
| `gazebo_sim.launch.py` starts | Gazebo Harmonic server + robot spawn complete without error |
| `/imu/data` exists | Published at ≥ 50 Hz after spawn |
| `/scan` exists | Published at ≥ 5 Hz after spawn |
| `/camera/image_raw` exists | Published at ≥ 10 Hz after spawn |

## Integration Tests

| Test | Expected Behavior |
|------|------------------|
| Full sim bringup | All sensor topics present within 30 s, Gazebo Harmonic responsive |

## Edge Cases

- Missing world file → Gazebo fails to start with clear error
- Missing URDF → robot spawn fails with clear error
