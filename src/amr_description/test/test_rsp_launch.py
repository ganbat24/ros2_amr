# Copyright 2026 Ganbat Selenge
#
# Licensed under the Apache License, Version 2.0 (the 'License');
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an 'AS IS' BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Tests for the robot state publisher launch file."""

import os
import unittest

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import launch_testing
import launch_testing.actions
from launch_testing_ros import WaitForTopics
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage
import xacro


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

    def test_tf_topic_exists(self):
        """Test that expected topics are published."""
        topic_list = [
            ('/tf', TFMessage),
            ('/joint_states', JointState),
        ]
        wait = WaitForTopics(topic_list, timeout=15.0)
        self.assertTrue(
            wait.wait(), 'Expected topics not found within timeout'
        )

    def test_xacro_processes(self):
        """Test that the xacro file processes correctly."""
        amr_description_pkg = get_package_share_directory('amr_description')
        xacro_path = os.path.join(
            amr_description_pkg, 'urdf', 'amr.urdf.xacro'
        )
        robot_description = xacro.process_file(xacro_path)
        self.assertIsNotNone(robot_description)
        self.assertIn('base_link', robot_description.toxml())
