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
"""amr_office world/map consistency tests.

The world SDF and the occupancy map are both generated from the floorplan
in amr_simulation/tools/generate_office_world.py. These tests:

1. Regenerate both artifacts and byte-compare against the installed files
   (guards against drift: editing the floorplan without regenerating).
2. Probe the occupancy grid at known coordinates so an axis flip or a
   misplaced wall fails loudly instead of silently breaking AMCL.
"""

import importlib.util
import os
import tempfile
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory

# --- load the generator as a plain module (it is a tool, not a package) ---
_TOOL_PATH = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    'tools',
    'generate_office_world.py',
)
_spec = importlib.util.spec_from_file_location('gen_office', _TOOL_PATH)
gen = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(gen)

# The world SDF is installed with this package (its share dir is on the
# AMENT_PREFIX_PATH when these tests run). The occupancy map is a source
# artifact of the SIBLING amr_navigation package — colcon test does not
# put sibling packages on the prefix path, so resolve it relative to the
# source tree (the map is generated and checked in, not built).
SHARE = get_package_share_directory('amr_simulation')
MAP_DIR = os.path.normpath(
    os.path.join(
        os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
        'amr_navigation',
        'maps',
    )
)


def _read(path):
    with open(path, 'rb') as f:
        return f.read()


def _pgm_grid():
    """Return (width, height, grid) with grid[py][px] byte values."""
    data = _read(os.path.join(MAP_DIR, 'amr_office.pgm'))
    lines = data.split(b'\n', 3)
    width, height = [int(x) for x in lines[1].split()]
    body = lines[3]
    grid = [
        [body[py * width + px] for px in range(width)]
        for py in range(height)
    ]
    return width, height, grid


def test_artifacts_regenerate_byte_identical():
    """The checked-in SDF/pgm/yaml must match a fresh generation."""
    with tempfile.TemporaryDirectory() as tmp:
        sdf_dir = os.path.join(tmp, 'sdf')
        map_dir = os.path.join(tmp, 'map')
        os.makedirs(sdf_dir)
        os.makedirs(map_dir)
        gen.main(['--sdf-out', sdf_dir, '--map-out', map_dir])

        assert _read(os.path.join(sdf_dir, 'amr_office.sdf')) == _read(
            os.path.join(SHARE, 'worlds', 'amr_office.sdf')
        ), 'amr_office.sdf drifted from the generator — re-run ' \
            'tools/generate_office_world.py'
        assert _read(os.path.join(map_dir, 'amr_office.pgm')) == _read(
            os.path.join(MAP_DIR, 'amr_office.pgm')
        ), 'amr_office.pgm drifted from the generator — re-run ' \
            'tools/generate_office_world.py'
        assert _read(os.path.join(map_dir, 'amr_office.yaml')) == _read(
            os.path.join(MAP_DIR, 'amr_office.yaml')
        ), 'amr_office.yaml drifted from the generator — re-run ' \
            'tools/generate_office_world.py'


def test_sdf_well_formed():
    tree = ET.parse(os.path.join(SHARE, 'worlds', 'amr_office.sdf'))
    world = tree.getroot().find('world')
    assert world is not None
    assert world.get('name') == 'amr_office'
    models = world.findall('model')
    names = {m.get('name') for m in models}
    # ground_plane + sun + 7 walls + 4 obstacles (O5 removed)
    assert len(models) == 13, names
    assert 'wall_00' in names and 'obstacle_00' in names


def test_map_orientation_and_layout():
    """Pixel probes: rows/cols must land on the expected structures.

    map_server convention: image row 0 = north, yaml origin = lower-left
    pixel. World y grows DOWNWARD in the file.
    """
    width, height, grid = _pgm_grid()
    assert width == 220 and height == 180, (width, height)

    def occupied(px, py):
        return grid[py][px] == 0

    def world_to_px(wx, wy):
        px = int((wx + gen.MARGIN) / gen.RES)
        py = int((gen.WORLD_Y + gen.MARGIN - wy) / gen.RES)
        return px, py

    # Outer walls: north (y 7.9..8.1), south (y -0.1..0.1), west, east.
    for wx, wy in [
        (5.0, 8.0), (5.0, 0.0), (0.0, 4.0), (10.0, 4.0),
        (2.5, 4.0),   # W1a (x 0..4.7)
        (8.0, 4.0),   # W1b (x 6.3..10)
        (7.8, 5.0),   # W2 (x 7.7..7.9, y 4.1..6.0)
        (3.5, 2.0),   # O1
        (8.5, 2.0),   # O2
        (1.8, 6.6),   # O3
        (8.7, 5.0),   # O4 (new position, y 4.6..5.3)
    ]:
        px, py = world_to_px(wx, wy)
        assert occupied(px, py), 'expected occupied at world (%g, %g)' % (
            wx, wy
        )

    # Free cells: spawn, door centers, goal poses.
    for wx, wy in [
        gen.SPAWN_POSE[:2],
        (5.5, 4.0),   # D1 door center (x 4.7..6.3)
        (7.0, 4.5),   # corridor left of W2
        (7.8, 7.0),   # W2 door into the top-right room (y 6.0..8.0)
        (9.4, 7.4),   # G1
        (3.2, 6.8),   # G2
        (9.3, 1.5),   # G3
    ]:
        px, py = world_to_px(wx, wy)
        assert grid[py][px] == 255, 'expected free at world (%g, %g)' % (
            wx, wy
        )
