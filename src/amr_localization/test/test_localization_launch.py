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
import yaml


def generate_test_description():
    return LaunchDescription(
        [
            launch_testing.actions.ReadyToTest(),
            TimerAction(period=float('inf'), actions=[]),
        ]
    )


class TestLocalizationLaunch(unittest.TestCase):
    def test_ekf_yaml_loads(self):
        amr_localization_pkg = get_package_share_directory('amr_localization')
        yaml_path = os.path.join(amr_localization_pkg, 'config', 'ekf.yaml')
        with open(yaml_path, 'r') as f:
            config = yaml.safe_load(f)
        self.assertIn('ekf_filter_node', config)
