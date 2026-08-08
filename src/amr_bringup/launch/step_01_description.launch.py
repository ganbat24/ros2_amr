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
Step 1 — Robot Description only (no Gazebo, no control).

Starts robot_state_publisher only (no joint_state_publisher since
there is no simulator or controller to provide joint states).
Use joint_state_publisher_gui if you want to manually set joints
in RViz for URDF inspection.

Verify:
  ros2 topic echo /robot_description --once
  ros2 run tf2_tools view_frames
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    amr_description_pkg = FindPackageShare('amr_description').find(
        'amr_description'
    )

    rsp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(amr_description_pkg, 'launch', 'rsp.launch.py')
        ),
        launch_arguments={
            'robot_description': os.path.join(
                amr_description_pkg, 'urdf', 'amr.urdf.xacro'
            ),
            'use_joint_state_publisher': LaunchConfiguration(
                'use_joint_state_publisher'
            ),
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name='use_joint_state_publisher',
                default_value='false',
                description='Launch joint_state_publisher (default false — '
                'only enable for standalone URDF inspection)',
            ),
            rsp_launch,
        ]
    )
