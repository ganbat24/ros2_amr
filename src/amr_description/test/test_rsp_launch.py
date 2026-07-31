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
"""Tests for the robot state publisher launch file."""

import os
import tempfile
import time
import unittest

from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node as RclpyNode
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.substitutions import FindPackageShare

import launch_testing
import launch_testing.actions

from sensor_msgs.msg import JointState

from tf2_msgs.msg import TFMessage

import xacro

STATIC_TF_QOS = QoSProfile(
    depth=10,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    reliability=QoSReliabilityPolicy.RELIABLE,
)


def generate_test_description():
    """Generate the test description for the robot state publisher."""
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
            )
        }.items(),
    )
    return LaunchDescription(
        [
            rsp_launch,
            launch_testing.actions.ReadyToTest(),
        ]
    )


class TestAmrDescriptionLaunch(unittest.TestCase):
    """Test cases for the AMR description launch."""

    def _make_node(self, name):
        ctx = rclpy.Context()
        rclpy.init(context=ctx)
        node = RclpyNode(node_name=name, context=ctx)
        return node, SingleThreadedExecutor(context=ctx), ctx

    def _shutdown_node(self, node, executor, ctx):
        node.destroy_node()
        rclpy.shutdown(context=ctx)

    def test_topics_and_rate(self):
        """Test /joint_states and /tf_static exist, rate >= 0.5 Hz."""
        node, executor, ctx = self._make_node('test_combined_node')
        executor.add_node(node)

        joint_timestamps = []
        tf_static_child_frames = set()
        tf_static_parent_map = {}

        def joint_callback(msg):
            joint_timestamps.append(
                msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            )

        def tf_static_callback(msg):
            for transform in msg.transforms:
                tf_static_child_frames.add(transform.child_frame_id)
                tf_static_parent_map[transform.child_frame_id] = (
                    transform.header.frame_id
                )

        sub_joint = node.create_subscription(
            JointState, '/joint_states', joint_callback, 10
        )
        sub_tf = node.create_subscription(
            TFMessage, '/tf_static', tf_static_callback, STATIC_TF_QOS
        )

        start = time.time()
        while time.time() - start < 5.0:
            executor.spin_once(timeout_sec=0.1)

        node.destroy_subscription(sub_joint)
        node.destroy_subscription(sub_tf)
        self._shutdown_node(node, executor, ctx)

        self.assertGreater(
            len(joint_timestamps), 0, '/joint_states not received'
        )
        self.assertGreater(
            len(tf_static_child_frames), 0, '/tf_static not received'
        )

        if len(joint_timestamps) >= 2:
            duration = joint_timestamps[-1] - joint_timestamps[0]
            if duration > 0:
                rate = (len(joint_timestamps) - 1) / duration
                self.assertGreaterEqual(
                    rate, 0.5, f'Joint states rate {rate:.1f} Hz < 0.5 Hz'
                )

        expected_children = {
            'base_link',
            'camera_link',
            'imu_link',
            'laser_frame',
            'laser_stand_link',
            'caster_front_link',
            'caster_rear_link',
        }
        missing = expected_children - tf_static_child_frames
        self.assertEqual(
            missing, set(), f'Missing frames in /tf_static: {missing}'
        )

        self.assertIn('base_link', tf_static_parent_map)
        self.assertEqual(
            tf_static_parent_map['base_link'],
            'base_footprint',
            'base_link parent should be base_footprint',
        )


class TestUrdfValidation(unittest.TestCase):
    """Build-time tests for URDF validation."""

    def test_check_urdf(self):
        """URDF xacro processes correctly and contains expected elements."""
        amr_description_pkg = get_package_share_directory('amr_description')
        xacro_path = os.path.join(
            amr_description_pkg, 'urdf', 'amr.urdf.xacro'
        )
        robot_description = xacro.process_file(xacro_path)
        urdf_xml = robot_description.toxml()

        self.assertIn('base_link', urdf_xml, 'base_link not found in URDF')
        self.assertIn(
            'wheel_left_link', urdf_xml, 'wheel_left_link not found in URDF'
        )
        self.assertIn(
            'wheel_right_link', urdf_xml, 'wheel_right_link not found in URDF'
        )
        self.assertIn('imu_link', urdf_xml, 'imu_link not found in URDF')
        self.assertIn('laser_frame', urdf_xml, 'laser_frame not found in URDF')
        self.assertIn('camera_link', urdf_xml, 'camera_link not found in URDF')
        self.assertIn(
            'base_footprint', urdf_xml, 'base_footprint not found in URDF'
        )

    def test_unknown_xacro_macro(self):
        """Xacro fails with clear error on unknown macro."""
        with tempfile.NamedTemporaryFile(
            mode='w', suffix='.xacro', delete=False
        ) as f:
            f.write('<?xml version="1.0"?>\n')
            f.write(
                '<robot name="test" xmlns:xacro='
                '"http://ros.org/wiki/xacro">\n'
            )
            f.write('  <xacro:undefined_macro />\n')
            f.write('</robot>\n')
            temp_path = f.name

        try:
            with self.assertRaises(Exception):
                xacro.process_file(temp_path)
        finally:
            os.unlink(temp_path)


if __name__ == '__main__':
    unittest.main()
