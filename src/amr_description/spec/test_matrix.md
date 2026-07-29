# amr_description — Test Matrix

## Build-Time Tests

| Test | Expected Behavior |
|------|------------------|
| `xacro` processes `amr.urdf.xacro` | Output valid XML, no undefined macros |
| URDF passes `check_urdf` | No joint/link errors, all frames reachable from `base_link` |

## Runtime Tests

| Test | Expected Behavior |
|------|------------------|
| `rsp.launch.py` starts without error | `robot_state_publisher` and `joint_state_publisher_gui` nodes present |
| `/tf` exists | All expected frames published within 5 s of launch |
| `/tf_static` exists | `laser_frame`, `imu_link`, `camera_optical_frame` present |
| `/joint_states` exists | Published at ≥ 10 Hz |
| TF topology matches plan | `map → odom → base_link → child` structure consistent |

## Integration Tests

| Test | Expected Behavior |
|------|------------------|
| `rsp.launch.py` + `gazebo_sim.launch.py` | TF tree complete, no frame lookup timeouts |

## Edge Cases

- `robot_description` parameter missing → launch fails with clear error
- URDF contains unknown macro → xacro fails during launch
