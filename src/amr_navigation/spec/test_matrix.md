# amr_navigation — Test Matrix

## Build-Time Tests

| Test | Expected Behavior |
|------|------------------|
| YAML loads via `yaml.safe_load` | No parse errors |
| `nav2_params.yaml` keys | All plugin names (`GridBased`, `FollowPath`, etc.) present |

## Runtime Tests

| Test | Expected Behavior |
|------|------------------|
| `navigation.launch.py` starts | All lifecycle nodes present |
| `/cmd_vel` produced | Published when goal received |
| `/global_costmap/costmap` | Published within 5 s |
| `/local_costmap/costmap` | Published within 5 s |

## Integration Tests

| Test | Expected Behavior |
|------|------------------|
| Full Nav2 bringup | Lifecycle transitions complete, BT navigator active |
| `navigate_to_pose` action | Produces valid path, local planner follows |

## Edge Cases

- Missing `/scan` topic → costmap not updated (expected)
- Wrong plugin name → lifecycle transition fails (forbidden by spec)
