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
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    amr_control_pkg = FindPackageShare('amr_control').find('amr_control')
    amr_description_pkg = FindPackageShare('amr_description').find(
        'amr_description'
    )
    controller_manager_config = os.path.join(
        amr_control_pkg, 'config', 'controller_manager.yaml'
    )

    default_model_path = os.path.join(
        amr_description_pkg, 'urdf', 'amr.urdf.xacro'
    )

    rsp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(amr_description_pkg, 'launch', 'rsp.launch.py')
        ),
        launch_arguments={'robot_description': default_model_path}.items(),
    )

    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[controller_manager_config],
        output='screen',
        remappings=[('/cmd_vel', '/cmd_vel')],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name='config_file',
                default_value=controller_manager_config,
                description='Absolute path to controller_manager YAML config',
            ),
            DeclareLaunchArgument(
                name='robot_description',
                default_value=default_model_path,
                description='Absolute path to the robot '
                'description xacro file',
            ),
            rsp_launch,
            controller_manager_node,
        ]
    )
