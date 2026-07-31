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
# limitations under the limitations under the limitations under the
# limitations.
"""Integration tests for amr_description with Gazebo simulation (headless)."""

import os
import shutil
import time
import unittest

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.substitutions import FindPackageShare

import launch_testing
import launch_testing.actions

from tf2_msgs.msg import TFMessage

STATIC_TF_QOS = QoSProfile(
    depth=10,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    reliability=QoSReliabilityPolicy.RELIABLE,
)


def generate_test_description():
    """Generate test description with rsp + gazebo headless."""
    amr_description_pkg = FindPackageShare('amr_description').find(
        'amr_description'
    )
    amr_simulation_pkg = get_package_share_directory('amr_simulation')

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


_has_gz = shutil.which('gz') is not None
try:
    get_package_share_directory('amr_simulation')
    _has_sim_pkg = True
except Exception:
    _has_sim_pkg = False


@unittest.skipUnless(
    _has_gz and _has_sim_pkg,
    'Gazebo (gz) or amr_simulation package not available',
)
class TestRspGazeboIntegration(unittest.TestCase):
    """Integration tests for rsp + gazebo (headless)."""

    def test_tf_static_exists(self):
        """Test that /tf_static is published with Gazebo running."""
        ctx = rclpy.Context()
        rclpy.init(context=ctx)
        node = rclpy.create_node('test_gazebo_tf', context=ctx)
        executor = SingleThreadedExecutor(context=ctx)
        executor.add_node(node)

        child_frames = set()

        def callback(msg):
            for transform in msg.transforms:
                child_frames.add(transform.child_frame_id)

        sub = node.create_subscription(
            TFMessage, '/tf_static', callback, STATIC_TF_QOS
        )

        start = time.time()
        while time.time() - start < 10.0:
            executor.spin_once(timeout_sec=0.1)

        node.destroy_subscription(sub)
        node.destroy_node()
        rclpy.shutdown(context=ctx)

        expected_frames = {
            'base_link',
            'imu_link',
            'laser_frame',
            'camera_link',
        }
        missing = expected_frames - child_frames
        self.assertEqual(
            missing, set(), f'Missing frames in /tf_static: {missing}'
        )
