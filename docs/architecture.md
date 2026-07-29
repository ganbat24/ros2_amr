# AMR Base Stack — Architecture Overview

This document provides the system-level data flow for the `amr_ws` stack.

---

## Package Map

```
amr_ws/
  src/
    amr_description/       # URDF/Xacro, ros2_control, Gazebo sensor plugins
    amr_control/           # diff_drive_controller config, controller_manager, C++ nodes
    amr_sensors/           # ros_gz_bridge launch + sensor bridge config
    amr_simulation/        # Gazebo Harmonic world, ros_gz_sim spawn, bridge remappings
    amr_localization/      # EKF, SLAM Toolbox, AMCL launch/config
    amr_navigation/        # Nav2 params, costmaps, planners, BT, maps
    amr_bringup/           # Top-level system.launch.py orchestrator
  docs/                    # Architecture docs
  docker/                  # Dockerfile, docker-compose
  .devcontainer/           # VS Code Dev Container
```

---

## System Diagram

```mermaid
graph TB
    subgraph Sim["Gazebo Harmonic"]
        WORLD["World SDF"]
        GZ_SENSORS["LiDAR / IMU / Camera<br/>gz transport topics"]
        GZ_CTRL["gz_ros2_control"]
    end

    subgraph Bridge["Sensor Bridging"]
        GZ_BRIDGE["ros_gz_bridge nodes<br/>(amr_sensors)"]
    end

    subgraph Robot["Robot Model"]
        DESC["amr_description<br/>URDF/Xacro"]
        DESC --> GZ_SENSORS
    end

    subgraph Control["Control"]
        CTRL["diff_drive_controller<br/>(amr_control)"]
        CTRL --> GZ_CTRL
    end

    subgraph Localization["Localization"]
        EKF["robot_localization EKF<br/>(amr_localization)"]
        AMCL["Nav2 AMCL"]
        SLAM["SLAM Toolbox"]
    end

    subgraph Navigation["Navigation"]
        COSTMAP["Nav2 Costmaps"]
        PLANNER["SMAC 2D Planner"]
        CONTROLLER["DWB Local Planner"]
        BT["bt_navigator"]
    end

    subgraph Orchestration["Bringup"]
        UP["amr_bringup/system.launch.py"]
    end

    GZ_SENSORS -->|/scan| GZ_BRIDGE
    GZ_SENSORS -->|/imu/data| GZ_BRIDGE
    GZ_SENSORS -->|/camera/image_raw| GZ_BRIDGE
    GZ_BRIDGE -->|/scan| COSTMAP
    GZ_BRIDGE -->|/imu/data| EKF
    EKF -->|/odom + TF odom->base_link| AMCL
    SLAM -->|/map + TF map->odom| PLANNER
    AMCL -->|/map + TF map->odom| PLANNER
    COSTMAP --> PLANNER
    PLANNER --> CONTROLLER
    CONTROLLER -->|/cmd_vel| CTRL
    UP --> Sim
    UP --> Bridge
    UP --> Control
    UP --> Localization
    UP --> Navigation
```

---

## Data Flow Summary

| Signal | Source | Sink |
|---|---|---|
| `/cmd_vel` | Nav2 DWB controller | `diff_drive_controller` |
| `/scan` | Gazebo LiDAR via `ros_gz_bridge` | Nav2 costmap, SLAM Toolbox |
| `/imu/data` | Gazebo IMU via `ros_gz_bridge` | EKF |
| `/camera/image_raw` | Gazebo RGB camera via `ros_gz_bridge` | `image_proc` |
| `/odom` | EKF (wheel odom + IMU fused) | Nav2, AMCL |
| `/map` | SLAM Toolbox or `map_server` | Nav2 global costmap |
| `/tf` map→odom→base_link | SLAM Toolbox or AMCL + EKF | All consumers |
| `/tf` base_link→sensors | `robot_state_publisher` | All consumers |

---

## Reproducibility

- **Docker:** `docker compose -f docker/docker-compose.yml up sim`
- **Dev Container:** Open in VS Code → "Reopen in Container"
- **Native:** Source ROS 2 Jazzy, `colcon build`, `source install/setup.bash`
- **rosdep:** `rosdep install --from-paths src --ignore-src -r -y`
