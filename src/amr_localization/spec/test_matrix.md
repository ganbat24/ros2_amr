# amr_localization — Test Matrix

## Build-Time Tests

| Test | Expected Behavior |
|------|------------------|
| YAML loads via `yaml.safe_load` | No parse errors |
| `ekf.yaml` keys | `odom_frame_id`, `base_link_frame`, `world_frame` match plan |
| `amcl_params.yaml` keys | `base_frame_id: base_link`, `odom_frame_id: odom`, `map_frame: map` |

## Runtime Tests

| Test | Expected Behavior |
|------|------------------|
| `localization.launch.py` starts | EKF, SLAM, AMCL nodes present |
| `/tf` `map → odom` | Published within 10 s of launch |
| `/tf` `odom → base_link` | Published at ≥ 30 Hz |
| `/odom` topic | Published at ≥ 30 Hz |
| `/map` topic | Published when SLAM/AMCL initialized |

## Integration Tests

| Test | Expected Behavior |
|------|------------------|
| EKF + SLAM + AMCL running | TF tree stable, no frame jumps |
| `/scan` + `/imu/data` input | EKF output covariance decreases over time |

## Edge Cases

- Missing `/scan` topic → EKF/SLAM degrade gracefully
- Wrong frame IDs in YAML → TF lookup fails (forbidden by spec)
