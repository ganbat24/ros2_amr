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
Step 3 — Sensor Bridges (LiDAR, IMU, Camera).

Includes Step 2 (description + Gazebo with controllers) and the
amr_sensors launch: /scan, /imu/data, /camera/image_raw,
/camera/camera_info, image_proc rectification, the scan restamper, and
the gz-scoped sensor static transforms.

Sim args (world/headless/...) are set at step_02 or system.launch.py.

Requires: Steps 1–2.

Verify:
  - ros2 topic echo /scan --once         (laser scan data)
  - ros2 topic echo /imu/data --once     (IMU quaternion + angular vel)
  - ros2 topic echo /camera/image_raw --once  (camera image)
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    amr_sensors_pkg = FindPackageShare('amr_sensors').find('amr_sensors')

    step_02_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                os.path.dirname(os.path.abspath(__file__)),
                'step_02_gazebo.launch.py',
            )
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
    )

    sensors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(amr_sensors_pkg, 'launch', 'sensors.launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name='use_sim_time',
                default_value='true',
                description='Use simulation clock',
            ),
            step_02_launch,
            sensors_launch,
        ]
    )
