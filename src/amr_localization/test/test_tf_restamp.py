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
"""
Regression tests for tf_restamp.is_stale (builtin Time message handling).

Guard against the crash where subtracting two builtin_interfaces/Time
messages directly raised ``TypeError`` (rclpy Time only supports
subtracting its own Time/Duration classes) — which took the restamper
down on its first /tf message.
"""

import importlib.util
import os

from builtin_interfaces.msg import Time as BuiltinTime

import pytest

_SCRIPT = os.path.abspath(os.path.join(
    os.path.dirname(__file__), '..', 'scripts', 'tf_restamp.py'))


@pytest.fixture(scope='module')
def restamp():
    """Import the installed script as a module (it is not a package)."""
    spec = importlib.util.spec_from_file_location('tf_restamp', _SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _t(sec, nanosec=0):
    """Build a builtin_interfaces/Time message."""
    return BuiltinTime(sec=sec, nanosec=nanosec)


def test_stale_when_clock_well_ahead(restamp):
    assert restamp.is_stale(_t(10, 500_000_001), _t(10))


def test_fresh_stamp_not_stale(restamp):
    assert not restamp.is_stale(_t(10), _t(10))


def test_own_republish_not_stale(restamp):
    """The echo of this node's own output carries stamp == clock_now."""
    assert not restamp.is_stale(_t(10, 123_456_789), _t(10, 123_456_789))


def test_no_clock_never_stale(restamp):
    assert not restamp.is_stale(None, _t(10))


def test_future_stamp_not_stale(restamp):
    assert not restamp.is_stale(_t(10), _t(10, 1))


def test_boundary_requires_strictly_greater(restamp):
    assert not restamp.is_stale(_t(10, 500_000_000), _t(10))
    assert restamp.is_stale(_t(10, 500_000_001), _t(10))
