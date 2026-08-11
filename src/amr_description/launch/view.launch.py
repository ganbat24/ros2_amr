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
"""Launch file for RViz visualization of the AMR."""

import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate the launch description for RViz visualization."""
    amr_description_pkg = FindPackageShare('amr_description').find(
        'amr_description'
    )

    rsp_launch_path = os.path.join(
        amr_description_pkg, 'launch', 'rsp.launch.py'
    )
    rsp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rsp_launch_path),
        launch_arguments={
            'robot_description': os.path.join(
                amr_description_pkg, 'urdf', 'amr.urdf.xacro'
            ),
            # Standalone inspection: no /clock source, and joint_state_
            # publisher (GUI) lets joints be moved by hand in RViz.
            'use_joint_state_publisher': 'true',
            'use_sim_time': 'false',
        }.items(),
    )

    rviz_config = os.path.join(
        amr_description_pkg, 'config', 'view_robot.rviz'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
    )

    return LaunchDescription(
        [
            rsp_launch,
            rviz_node,
        ]
    )
