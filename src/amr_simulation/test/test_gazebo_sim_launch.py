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


class TestGazeboSimLaunch(unittest.TestCase):
    def test_artifacts_exist(self):
        """World SDF, GUI config, and launch file are installed."""
        amr_simulation_pkg = get_package_share_directory('amr_simulation')
        for rel in (
            'worlds/empty_world.sdf',
            'gazebo/gui_no_quickstart.config',
            'launch/gazebo_sim.launch.py',
        ):
            self.assertTrue(
                os.path.isfile(os.path.join(amr_simulation_pkg, rel)),
                f'Missing installed artifact: {rel}',
            )

    def test_world_sdf_parses(self):
        """empty_world.sdf is well-formed XML."""
        import xml.etree.ElementTree as ET

        amr_simulation_pkg = get_package_share_directory('amr_simulation')
        world_path = os.path.join(
            amr_simulation_pkg, 'worlds', 'empty_world.sdf'
        )
        ET.parse(world_path)  # raises on malformed XML
