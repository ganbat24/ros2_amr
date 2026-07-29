# amr_bringup — Dependencies

## Upstream (Consumed)

| Package | Type | Consumed By |
|---------|------|-------------|
| `amr_description` | launch | `rsp.launch.py` included |
| `amr_control` | launch | `controller_manager.launch.py` included |
| `amr_simulation` | launch | `gazebo_sim.launch.py` included |
| `amr_sensors` | launch | `sensors.launch.py` included |
| `amr_localization` | launch | `localization.launch.py` included |
| `amr_navigation` | launch | `navigation.launch.py` included |

## Downstream (Produced)

None owned directly.

## External Dependencies

- `launch`
- `launch_ros`
