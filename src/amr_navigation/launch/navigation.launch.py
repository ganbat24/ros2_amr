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
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    amr_navigation_pkg = FindPackageShare('amr_navigation').find(
        'amr_navigation'
    )
    nav2_bringup_pkg = FindPackageShare('nav2_bringup').find('nav2_bringup')

    default_nav_params = os.path.join(
        amr_navigation_pkg, 'config', 'nav2_params.yaml'
    )
    default_bt_xml_path = os.path.join(
        nav2_bringup_pkg,
        'behavior_trees',
        'navigate_w_replanning_and_recovery.xml',
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    bt_xml_filename = LaunchConfiguration('bt_xml_filename')

    # Rewrite the Nav2 YAML at launch time so that use_sim_time and the
    # behavior tree are taken from launch arguments.
    nav2_params = RewrittenYaml(
        source_file=params_file,
        root_key='',
        param_rewrites={
            'use_sim_time': use_sim_time,
            'default_bt_xml_filename': bt_xml_filename,
        },
        convert_types=True,
    )

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        output='screen',
        parameters=[nav2_params],
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params],
    )

    smoother_server = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[nav2_params],
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_params],
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_params],
    )

    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[nav2_params],
    )

    # nav2_velocity_smoother outputs plain Twist on /cmd_vel_smoothed but
    # ros2_controllers 4.x diff_drive subscribes TwistStamped (on
    # /cmd_vel_stamped — see the spawner remap in gazebo_sim.launch.py).
    twist_to_stamped = Node(
        package='amr_navigation',
        executable='twist_to_stamped.py',
        name='twist_to_stamped',
        output='log',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Start the lifecycle manager AFTER the discovery storm settles: its
    # service clients created during the 40-node launch storm hang (the
    # planner_server activation call times out after ~2 min and the manager
    # aborts bringup, leaving every nav node inactive). A 60 s delay lets
    # discovery settle on constrained hosts; on fast hosts the nodes are
    # simply activated a minute later.
    lifecycle_manager = TimerAction(
        period=60.0,
        actions=[
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager',
                output='screen',
                parameters=[nav2_params],
            )
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name='params_file',
                default_value=default_nav_params,
                description='Nav2 parameter YAML file',
            ),
            DeclareLaunchArgument(
                name='use_sim_time',
                default_value='true',
                description='Use simulation clock',
            ),
            DeclareLaunchArgument(
                name='bt_xml_filename',
                default_value=default_bt_xml_path,
                description='Behavior tree XML file',
            ),
            controller_server,
            planner_server,
            smoother_server,
            behavior_server,
            bt_navigator,
            velocity_smoother,
            twist_to_stamped,
            lifecycle_manager,
        ]
    )
