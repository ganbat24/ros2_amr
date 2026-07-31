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
"""Integration tests for amr_description with Gazebo simulation."""

import os
import shutil
import time
import unittest

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.substitutions import FindPackageShare

import launch_testing
import launch_testing.actions

import rclpy
from rclpy.executors import SingleThreadedExecutor

import tf2_ros


def generate_test_description():
    """Generate test description with rsp + gazebo headless."""
    amr_description_pkg = FindPackageShare('amr_description').find(
        'amr_description'
    )
    amr_simulation_pkg = FindPackageShare('amr_simulation').find(
        'amr_simulation'
    )

    rsp_launch_path = os.path.join(
        amr_description_pkg, 'launch', 'rsp.launch.py'
    )

    rsp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rsp_launch_path),
        launch_arguments={
            'robot_description': os.path.join(
                amr_description_pkg, 'urdf', 'amr.urdf.xacro'
            )
        }.items(),
    )

    world_path = os.path.join(amr_simulation_pkg, 'worlds', 'empty_world.sdf')

    gazebo_server = ExecuteProcess(
        cmd=['gz', 'sim', '-s', '-r', world_path],
        output='screen',
    )

    spawn_entity = ExecuteProcess(
        cmd=[
            'ros2',
            'run',
            'ros_gz_sim',
            'create',
            '-topic',
            '/robot_description',
            '-entity',
            'amr',
            '-z',
            '0.1',
        ],
        output='screen',
    )

    return LaunchDescription(
        [
            rsp_launch,
            gazebo_server,
            spawn_entity,
            launch_testing.actions.ReadyToTest(),
        ]
    )


@unittest.skipUnless(
    shutil.which('gz') is not None,
    'Gazebo (gz) not available in PATH',
)
class TestRspGazeboIntegration(unittest.TestCase):
    """Integration tests for rsp + gazebo."""

    def test_tf_tree(self):
        """Test TF tree has expected frames with Gazebo running."""
        ctx = rclpy.Context()
        rclpy.init(context=ctx)
        node = rclpy.create_node('test_gazebo_tf', context=ctx)
        executor = SingleThreadedExecutor(context=ctx)
        executor.add_node(node)

        tf_buffer = tf2_ros.Buffer()
        _listener = tf2_ros.TransformListener(tf_buffer, node)  # noqa: F841

        expected_transforms = [
            ('base_footprint', 'base_link'),
            ('base_link', 'imu_link'),
            ('base_link', 'laser_frame'),
            ('base_link', 'camera_link'),
        ]

        start = time.time()
        while time.time() - start < 10.0:
            executor.spin_once(timeout_sec=0.1)

        node.destroy_node()
        rclpy.shutdown(context=ctx)

        for parent, child in expected_transforms:
            self.assertTrue(
                tf_buffer.can_transform(parent, child, rclpy.time.Time()),
                f'Missing transform: {parent} -> {child}',
            )
