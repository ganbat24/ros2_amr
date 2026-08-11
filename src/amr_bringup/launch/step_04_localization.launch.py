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
Step 4 — Localization (EKF + SLAM Toolbox or AMCL).

Includes Step 3 (description + Gazebo + sensors) and the amr_localization
launch: robot_localization EKF plus either SLAM Toolbox (use_slam=true)
or AMCL + map_server (use_slam=false, default).

SLAM and AMCL are mutually exclusive — both publish map → odom.

Sim args (world/headless/...) are set at step_02 or system.launch.py.

Requires: Steps 1–3.

Verify:
  - ros2 topic echo /tf --once           (odom → base_link transform)
  - ros2 node list | grep ekf
  - ros2 node list | grep amcl (or slam)
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    amr_localization_pkg = FindPackageShare('amr_localization').find(
        'amr_localization'
    )
    amr_navigation_pkg = FindPackageShare('amr_navigation').find(
        'amr_navigation'
    )

    default_map_file = os.path.join(
        amr_navigation_pkg, 'maps', 'empty_map.yaml'
    )

    step_03_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                os.path.dirname(os.path.abspath(__file__)),
                'step_03_sensors.launch.py',
            )
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
    )

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                amr_localization_pkg, 'launch', 'localization.launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'use_slam': LaunchConfiguration('use_slam'),
            'map': LaunchConfiguration('map'),
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name='use_sim_time',
                default_value='true',
                description='Use simulation clock',
            ),
            DeclareLaunchArgument(
                name='use_slam',
                default_value='false',
                description='Use SLAM Toolbox for mapping (true) or AMCL '
                'for localization with pre-built map (false). Default AMCL: '
                'slam_toolbox 2.8.5 has an upstream params regression.',
            ),
            DeclareLaunchArgument(
                name='map',
                default_value=default_map_file,
                description='Map YAML file for AMCL/map_server mode',
            ),
            step_03_launch,
            localization_launch,
        ]
    )
