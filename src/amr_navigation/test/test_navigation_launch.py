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
import glob
import importlib.util
import os
import unittest
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from ament_index_python.resources import get_resource, has_resource
from launch import LaunchDescription
from launch.actions import TimerAction

import launch_testing.actions
import yaml


def _load_navigation_launch_module():
    """Import the installed navigation.launch.py as a module."""
    path = os.path.join(
        get_package_share_directory('amr_navigation'), 'launch',
        'navigation.launch.py')
    spec = importlib.util.spec_from_file_location('navigation_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


# dwb_core registers its classes under its own namespace, but this package
# pulls it in through the nav2_dwb_controller metapackage rather than by
# name. Every other plugin namespace here is also its package name.
PLUGIN_PACKAGE_ALIASES = {'dwb_core': 'nav2_dwb_controller'}


def _declared_dependencies():
    """Every package amr_navigation's installed package.xml declares."""
    path = os.path.join(
        get_package_share_directory('amr_navigation'), 'package.xml')
    root = ET.parse(path).getroot()
    return {element.text.strip()
            for tag in ('depend', 'build_depend', 'exec_depend')
            for element in root.iter(tag) if element.text}


def _plugin_classes(node):
    """Every value of a `plugin:` key anywhere in a parsed params file."""
    found = []
    if isinstance(node, dict):
        for key, value in node.items():
            if key == 'plugin' and isinstance(value, str):
                found.append(value)
            else:
                found += _plugin_classes(value)
    elif isinstance(node, list):
        for value in node:
            found += _plugin_classes(value)
    return found


def _configured_plugins():
    """(params file, plugin class) for every shipped nav2 params file."""
    config_dir = os.path.join(
        get_package_share_directory('amr_navigation'), 'config')
    found = []
    for path in sorted(glob.glob(os.path.join(config_dir, '*.yaml'))):
        with open(path, 'r') as f:
            config = yaml.safe_load(f)
        for plugin in sorted(set(_plugin_classes(config))):
            found.append((os.path.basename(path), plugin))
    return found


def generate_test_description():
    return LaunchDescription(
        [
            launch_testing.actions.ReadyToTest(),
            TimerAction(period=float('inf'), actions=[]),
        ]
    )


class TestNavigationLaunch(unittest.TestCase):
    def test_nav2_params_yaml_loads(self):
        amr_navigation_pkg = get_package_share_directory('amr_navigation')
        yaml_path = os.path.join(
            amr_navigation_pkg, 'config', 'nav2_params.yaml'
        )
        with open(yaml_path, 'r') as f:
            config = yaml.safe_load(f)
        self.assertIn('bt_navigator', config)
        self.assertIn('controller_server', config)

    def test_composed_plugins_are_actually_registered(self):
        """
        Every composed plugin name must exist in the components index.

        This repo's recurring failure is a name that was never verified
        against the machine: wrong parameter names are silently ignored, and
        a wrong plugin name here would only surface as a failed component
        load partway through a 15-minute sim run. The registry is a plain
        ament index resource, so the check costs milliseconds offline.

        It also pins the one name that does not follow the pattern:
        nav2_behaviors registers behavior_server::BehaviorServer, not
        nav2_behaviors::BehaviorServer.
        """
        module = _load_navigation_launch_module()
        for package, plugin, _name in module.NAV2_COMPONENTS:
            self.assertTrue(
                has_resource('rclcpp_components', package),
                '%s registers no rclcpp_components at all' % package)
            registered, _ = get_resource('rclcpp_components', package)
            names = [line.split(';')[0]
                     for line in registered.splitlines() if line.strip()]
            self.assertIn(
                plugin, names,
                '%s is not registered by %s; it provides %s'
                % (plugin, package, names))

    def test_lifecycle_manager_is_composed_last(self):
        """
        Components load in order, so the manager must come last.

        The composed path has no settle timer: it relies on the lifecycle
        manager being constructed after the servers it manages, so it cannot
        create service clients for nodes that do not exist yet. If someone
        reorders this list, the composed stack regains the hang that the
        process-per-node path needs an 8 s timer to avoid.
        """
        module = _load_navigation_launch_module()
        self.assertEqual(module.NAV2_COMPONENTS[-1][2], 'lifecycle_manager')

    def test_composed_node_names_match_the_params_file(self):
        """
        A composed node whose name has no params block gets defaults.

        Composition changes nothing about how parameters are matched — the
        YAML is still keyed by node name — so a rename here would silently
        drop that server's entire configuration.
        """
        module = _load_navigation_launch_module()
        yaml_path = os.path.join(
            get_package_share_directory('amr_navigation'), 'config',
            'nav2_params.yaml')
        with open(yaml_path, 'r') as f:
            config = yaml.safe_load(f)
        for _package, _plugin, name in module.NAV2_COMPONENTS:
            self.assertIn(
                name, config,
                '%s is composed but nav2_params.yaml has no block for it, '
                'so it would run entirely on defaults' % name)

    def test_params_plugin_packages_are_declared(self):
        """
        Every plugin a params file names must come from a declared package.

        pluginlib plugins are loaded by string and linked against by nothing,
        so an undeclared one is present on any machine that installed the
        navigation2 metapackage and missing everywhere rosdep resolved these
        package.xml deps instead — which is CI. It then fails at the worst
        possible moment: the server cannot create the plugin, its configure
        fails, the lifecycle manager aborts bring-up, and the only symptom is
        that nothing ever reaches active.

        That is what promoting RegulatedPurePursuitController to the default
        cost on 2026-08-20: the nav smoke test spent four minutes waiting for
        lifecycle nodes to go active and timed out with nothing in the job log
        to say why. The check above covers the composed rclcpp_components;
        this one covers the params, for every variant file shipped, not just
        the default.
        """
        declared = _declared_dependencies()
        checked = _configured_plugins()
        self.assertTrue(checked, 'no plugins found — the scan is broken')
        for params_file, plugin in checked:
            package = plugin.split('::')[0]
            package = PLUGIN_PACKAGE_ALIASES.get(package, package)
            self.assertIn(
                package, declared,
                '%s loads %s, but amr_navigation does not declare %s, so '
                'rosdep never installs it in a clean environment'
                % (params_file, plugin, package))
