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
import unittest

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction

import launch_testing.actions


def generate_test_description():
    return LaunchDescription(
        [
            launch_testing.actions.ReadyToTest(),
            TimerAction(period=float('inf'), actions=[]),
        ]
    )


class TestSystemLaunch(unittest.TestCase):
    def test_all_launch_files_installed(self):
        """Every bringup launch file is present in the install space."""
        amr_bringup_pkg = get_package_share_directory('amr_bringup')
        for rel in (
            'launch/system.launch.py',
            'launch/step_01_description.launch.py',
            'launch/step_02_gazebo.launch.py',
            'launch/step_03_sensors.launch.py',
            'launch/step_04_localization.launch.py',
            'launch/step_05_navigation.launch.py',
        ):
            self.assertTrue(
                os.path.isfile(os.path.join(amr_bringup_pkg, rel)),
                f'Missing installed launch file: {rel}',
            )
