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
"""Launch file for the robot state publisher node."""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import xacro


def generate_launch_description():
    """Generate the launch description for the robot state publisher."""
    robot_description_pkg = FindPackageShare('amr_description').find(
        'amr_description'
    )
    default_model_path = os.path.join(
        robot_description_pkg, 'urdf', 'amr.urdf.xacro'
    )

    robot_description_config = xacro.process_file(default_model_path)
    robot_description = {'robot_description': robot_description_config.toxml()}

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description],
        output='screen',
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        parameters=[robot_description],
        output='screen',
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name='robot_description',
                default_value=default_model_path,
                description='Absolute path to the robot description xacro '
                'file',
            ),
            robot_state_publisher_node,
            joint_state_publisher_gui_node,
        ]
    )
