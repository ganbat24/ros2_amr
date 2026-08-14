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
Step 5 — Nav2 Navigation Stack.

Includes Step 4 (everything up to localization) and the amr_navigation
launch: controller_server, planner_server, smoother_server,
behavior_server, bt_navigator, velocity_smoother, the Twist->TwistStamped
relay, and the lifecycle manager.

Sim args (world/headless/...) are set at step_02 or system.launch.py.

Requires: Steps 1–4.

Verify:
  - ros2 node list | grep controller_server
  - ros2 lifecycle get /controller_server  (should be active)
  - ros2 action send_goal /navigate_to_pose ...
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    amr_navigation_pkg = FindPackageShare('amr_navigation').find(
        'amr_navigation'
    )
    # nav2_bringup ships no behavior_trees/ dir in this nav2 release; the
    # default tree lives in nav2_bt_navigator's own share dir (see
    # amr_navigation/launch/navigation.launch.py for the parameter-name
    # side of this).
    nav2_bt_navigator_pkg = FindPackageShare('nav2_bt_navigator').find(
        'nav2_bt_navigator'
    )

    default_nav_params = os.path.join(
        amr_navigation_pkg, 'config', 'nav2_params.yaml'
    )
    default_bt_xml_path = os.path.join(
        nav2_bt_navigator_pkg,
        'behavior_trees',
        'navigate_to_pose_w_replanning_and_recovery.xml',
    )

    step_04_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                os.path.dirname(os.path.abspath(__file__)),
                'step_04_localization.launch.py',
            )
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                amr_navigation_pkg, 'launch', 'navigation.launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': LaunchConfiguration('params_file'),
            'bt_xml_filename': LaunchConfiguration('bt_xml_filename'),
            'use_composition': LaunchConfiguration('use_composition'),
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
                name='params_file',
                default_value=default_nav_params,
                description='Nav2 parameter YAML file',
            ),
            DeclareLaunchArgument(
                name='bt_xml_filename',
                default_value=default_bt_xml_path,
                description='Behavior tree XML file',
            ),
            DeclareLaunchArgument(
                name='use_composition',
                default_value='false',
                description='Run the nav2 servers inside one component '
                'container instead of one process each.',
            ),
            step_04_launch,
            navigation_launch,
        ]
    )
