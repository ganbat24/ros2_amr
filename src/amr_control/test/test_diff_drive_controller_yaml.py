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

from ament_index_python.packages import get_package_share_directory
import yaml


def test_diff_drive_controller_yaml_loads():
    amr_control_pkg = get_package_share_directory('amr_control')
    yaml_path = os.path.join(
        amr_control_pkg, 'config', 'diff_drive_controller.yaml'
    )
    with open(yaml_path, 'r') as f:
        config = yaml.safe_load(f)
    # Config uses the namespace-agnostic '/**/' key accepted by the
    # controller_manager spawner --param-file.
    controller_key = next(
        k for k in config if k.endswith('diff_drive_controller')
    )
    assert controller_key == '/**/diff_drive_controller'
    params = config[controller_key]['ros__parameters']
    assert params['enable_odom_tf'] is False
    assert params['left_wheel_names'] == ['wheel_left_joint']
    assert params['right_wheel_names'] == ['wheel_right_joint']
    assert params['wheel_separation'] == 0.3
    assert params['wheel_radius'] == 0.033
    assert params['publish_rate'] == 50.0
