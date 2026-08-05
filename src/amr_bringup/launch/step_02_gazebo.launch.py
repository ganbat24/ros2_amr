# Copyright 2026 Ganbat Selenge
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Step 2 — Gazebo Harmonic + Robot Spawn + Controllers.

Opens Gazebo with the empty world, spawns the AMR, and the
gz_ros2_control plugin (GazeboSimROS2ControlPlugin) automatically
creates its own controller_manager and loads the controllers
defined in controller_manager.yaml.

No standalone ros2_control_node is needed — the Gazebo plugin
handles everything internally.

Requires: Step 1 (robot description).

Verify:
  - Gazebo window opens, robot appears on the ground plane
  - ros2 topic echo /clock --once        (sim time ticking)
  - ros2 topic echo /joint_states --once  (from Gazebo plugin)
  - ros2 topic echo /odom --once          (from diff_drive_controller)
  - ros2 control list_hardware_interfaces  (shows diff_drive)
  - ros2 control list_controllers          (shows loaded controllers)
  - ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.5}}"
    → robot moves forward
"""
import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    amr_simulation_pkg = FindPackageShare('amr_simulation').find(
        'amr_simulation'
    )

    default_world_path = os.path.join(
        amr_simulation_pkg, 'worlds', 'empty_world.sdf'
    )
    default_gui_config = os.path.join(
        amr_simulation_pkg, 'gazebo', 'gui_no_quickstart.config'
    )

    # Build gz_args with resolved paths (available at parse time)
    # Use -s for server-only (headless) to avoid WSL2 OpenGL issues
    gz_args_value = f'-s -r -v 1' f' {default_world_path}'

    # Include Step 1 (robot description)
    step_01_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                os.path.dirname(os.path.abspath(__file__)),
                'step_01_description.launch.py',
            )
        ),
    )

    # Launch Gazebo Harmonic
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                FindPackageShare('ros_gz_sim').find('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py',
            )
        ),
        launch_arguments={
            'gz_args': gz_args_value,
        }.items(),
    )

    # Spawn the robot into Gazebo from /robot_description topic
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic',
            '/robot_description',
            '-entity',
            'amr',
            '-x',
            '0.0',
            '-y',
            '0.0',
            '-z',
            '0.073',
        ],
        output='screen',
    )

    # Bridge /clock: Gazebo → ROS 2 (sim time)
    bridge_clock = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )

    # Get path to controller config
    amr_control_pkg = FindPackageShare('amr_control').find('amr_control')
    controller_config = os.path.join(
        amr_control_pkg, 'config', 'diff_drive_controller.yaml'
    )

    # Controller spawners — loaded after robot is spawned
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )

    diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'diff_drive_controller',
            '--param-file',
            controller_config,
        ],
    )

    return LaunchDescription(
        [
            # Fix Gazebo transport on WSL2 (multicast doesn't work)
            SetEnvironmentVariable('GZ_IP', '127.0.0.1'),
            DeclareLaunchArgument(
                name='world',
                default_value=default_world_path,
                description='Gazebo world SDF file',
            ),
            DeclareLaunchArgument(
                name='gui_config',
                default_value=default_gui_config,
                description='Gazebo GUI config (no quick start)',
            ),
            step_01_launch,
            gazebo,
            spawn_entity,
            bridge_clock,
            # Load joint_state_broadcaster after spawn completes
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=spawn_entity,
                    on_exit=[joint_state_broadcaster_spawner],
                )
            ),
            # Load diff_drive_controller after joint_state_broadcaster
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=joint_state_broadcaster_spawner,
                    on_exit=[diff_drive_controller_spawner],
                )
            ),
        ]
    )
