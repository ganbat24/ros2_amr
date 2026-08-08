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
"""
Step 2 — Gazebo Harmonic + Robot Spawn + Controllers.

Opens Gazebo (server-only by default, -s, to avoid WSL2 OpenGL issues),
spawns the AMR, and loads joint_state_broadcaster + diff_drive_controller
via spawners after the robot appears. The gz_ros2_control plugin
(GazeboSimROS2ControlPlugin) automatically creates its own
controller_manager; no standalone ros2_control_node is needed.

Requires: Step 1 (robot description).

Verify:
  - ros2 topic echo /clock --once        (sim time ticking)
  - ros2 topic echo /joint_states --once  (from Gazebo plugin)
  - ros2 topic echo /odom --once          (from diff_drive_controller)
  - ros2 control list_controllers         (shows loaded controllers)
  - ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.5}}"
    → robot moves forward
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    amr_simulation_pkg = FindPackageShare('amr_simulation').find(
        'amr_simulation'
    )

    default_world_path = os.path.join(
        amr_simulation_pkg, 'worlds', 'empty_world.sdf'
    )

    # Include Step 1 (robot description)
    step_01_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                os.path.dirname(os.path.abspath(__file__)),
                'step_01_description.launch.py',
            )
        ),
    )

    # gazebo_sim.launch.py brings up Gazebo, spawns the robot, bridges
    # /clock, and loads the controllers. Headless by default (-s) to
    # avoid WSL2 OpenGL issues.
    gazebo_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                amr_simulation_pkg, 'launch', 'gazebo_sim.launch.py'
            )
        ),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'paused': LaunchConfiguration('paused'),
            'verbose': LaunchConfiguration('verbose'),
            'headless': LaunchConfiguration('headless'),
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name='world',
                default_value=default_world_path,
                description='Gazebo world SDF file',
            ),
            DeclareLaunchArgument(
                name='paused',
                default_value='false',
                description='Start Gazebo paused',
            ),
            DeclareLaunchArgument(
                name='verbose',
                default_value='1',
                description='Gazebo verbose output level (0-4)',
            ),
            DeclareLaunchArgument(
                name='headless',
                default_value='true',
                description='Run Gazebo server-only (no GUI)',
            ),
            step_01_launch,
            gazebo_sim_launch,
        ]
    )
