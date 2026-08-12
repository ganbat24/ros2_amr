#!/usr/bin/env python3
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
"""Generate the amr_office world SDF and occupancy map from one floorplan.

The floorplan lives HERE (RECTS below) — both artifacts derive from it,
so world geometry and the occupancy map are consistent by construction.
Re-run to regenerate after editing the floorplan:

    python3 tools/generate_office_world.py

Default outputs (relative to the script location):
    ../worlds/amr_office.sdf
    ../../amr_navigation/maps/amr_office.pgm
    ../../amr_navigation/maps/amr_office.yaml

The robot spawns at SPAWN_POSE; AMCL's initial pose (launch args
initial_x/initial_y/initial_yaw) must match it.
"""

import argparse
import os
import sys

# ---------------------------------------------------------------------------
# Floorplan — single source of truth.
#
# World frame: x east, y north, origin at the outer wall's inner corner.
# Rects are (xmin, ymin, xmax, ymax, height) with the wall/box footprint.
# Keep everything axis-aligned; doors are the gaps between wall segments.
# ---------------------------------------------------------------------------

WALL_T = 0.2   # wall thickness (m)
WALL_H = 1.2   # wall height (m)
OBST_H = 0.8   # obstacle box height (m)
RES = 0.05     # map resolution (m/px)
MARGIN = 0.5   # map margin around the outer walls (m)

WORLD_X = 10.0  # outer wall inner extents
WORLD_Y = 8.0

SPAWN_POSE = (1.5, 1.5, 0.0)  # (x, y, yaw) — robot spawn + AMCL initial pose

# Named goal poses used by the long-run validation scenario (map frame).
# All goals keep >=0.35 m clearance to obstacles/walls for the 0.25 m
# footprint radius.
GOALS = {
    "g1_top_right": (9.4, 7.4, 0.0),
    "g2_top_left": (3.2, 6.8, 0.0),
    "g3_bottom_right": (9.3, 1.5, 0.0),
    "g4_home": (1.5, 1.5, 0.0),
}

# Wall segments (axis-aligned rects). Outer walls centered on the 0/10 and
# 0/8 lines; interior walls leave door gaps. Doors are >=1.4 m wide: the
# robot footprint is 0.5 m and the costmap inflation_radius is 0.5 m, so a
# 1.2 m door left only ~0.2 m of low-cost band and DWB wedged the robot
# into the wall ends ("Start occupied" replan failures).
WALLS = [
    # outer south / north / west / east
    (0.0, -WALL_T / 2, WORLD_X, WALL_T / 2, WALL_H),
    (0.0, WORLD_Y - WALL_T / 2, WORLD_X, WORLD_Y + WALL_T / 2, WALL_H),
    (-WALL_T / 2, 0.0, WALL_T / 2, WORLD_Y, WALL_H),
    (WORLD_X - WALL_T / 2, 0.0, WORLD_X + WALL_T / 2, WORLD_Y, WALL_H),
    # W1a/W1b: split bottom rooms, door D1 x 4.4..6.6 (2.2 m)
    (0.0, 3.9, 4.4, 4.1, WALL_H),
    (6.6, 3.9, 10.0, 4.1, WALL_H),
    # W2: split top; spans y 4.1..6.0 only — the y 6.0..8.0 gap (2 m) is
    # the door into the top-right room. (A wall spanning to y=8.0 sealed
    # the room against the north wall: G1 was unreachable, "no valid
    # path" from any start.)
    (7.7, 4.1, 7.9, 6.0, WALL_H),
]

# Obstacle boxes (xmin, ymin, xmax, ymax, height).
OBSTACLES = [
    (3.1, 1.6, 3.9, 2.4, OBST_H),        # O1 — bottom-left room
    (8.15, 1.65, 8.85, 2.35, OBST_H),    # O2 — bottom-right room
    (1.5, 6.3, 2.1, 6.9, OBST_H),        # O3 — top-left room
    (8.4, 4.6, 9.1, 5.3, OBST_H),        # O4 — top-right room (kept clear
                                         # of the W2 door entry at x 7.9)
    (5.25, 5.25, 5.75, 5.75, OBST_H),    # O5 — corridor pinch
]

# ---------------------------------------------------------------------------
# World SDF template
# ---------------------------------------------------------------------------

_SDF_TEMPLATE = """<?xml version="1.0"?>
<sdf version="1.7">
  <world name="amr_office">
    <physics type="dart">
      <max_step_size>0.01</max_step_size>
      <!-- RTF 0.5: on this 2-vCPU + software-rendering host the physics
           outruns AMCL/costmaps at 1.0, producing TF-vs-scan timestamp
           skew that smears obstacle transforms around a turning robot
           (localization bias, "Start occupied", DWB no-legal-trajectory).
           Slowing the sim lets the perception stack keep up. -->
      <real_time_factor>0.5</real_time_factor>
    </physics>
    <gravity>0 0 -9.81</gravity>

    <scene>
      <ambient>0.4 0.4 0.4 1.0</ambient>
      <background>0.7 0.7 0.7 1.0</background>
      <grid>true</grid>
    </scene>

    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <latitude_deg>0.0</latitude_deg>
      <longitude_deg>0.0</longitude_deg>
      <elevation>0.0</elevation>
      <heading_deg>0</heading_deg>
    </spherical_coordinates>

    <model name="ground_plane">
      <static>true</static>
      <link name="ground_plane_link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
            </plane>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>1.0</mu>
                <mu2>1.0</mu2>
              </ode>
            </friction>
          </surface>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1.0</ambient>
            <diffuse>0.8 0.8 0.8 1.0</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <model name="sun">
      <static>true</static>
      <link name="sun_link">
        <light name="sun_light" type="directional">
          <cast_shadows>true</cast_shadows>
          <pose>0 0 10 0 0 0</pose>
          <diffuse>0.8 0.8 0.8 1</diffuse>
          <specular>0.2 0.2 0.2 1</specular>
          <attenuation>
            <range>1000</range>
            <constant>0.9</constant>
            <linear>0.01</linear>
            <quadratic>0.001</quadratic>
          </attenuation>
          <direction>-0.5 0.1 -0.9</direction>
        </light>
      </link>
    </model>

%(models)s
    <!-- gz-sim8 loads the DEFAULT server config (which includes the
         Physics system) ONLY when the world SDF declares no plugins.
         Once any <plugin> is declared, only those listed load — so the
         physics system must be declared explicitly or the world runs
         with no physics engine (bodies never fall, joints never move,
         joint-state components stay empty). -->
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics">
    </plugin>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands">
    </plugin>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster">
    </plugin>
    <!-- Required for any <sensor> in spawned models: without the sensor
         system the LiDAR/IMU/camera entities exist but never publish. -->
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
    </plugin>
    <!-- IMU sensors are driven by gz-sim-imu-system, NOT by the Sensors
         system (which only handles rendering-based sensors: camera,
         lidar). Without this plugin an <sensor type="imu"> entity exists
         in the ECM but never publishes. -->
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu">
    </plugin>
  </world>
</sdf>
"""

_MODEL_TEMPLATE = """    <model name="%(name)s">
      <static>true</static>
      <pose>%(cx)f %(cy)f %(hz)f 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>%(sx)f %(sy)f %(h)f</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>%(sx)f %(sy)f %(h)f</size>
            </box>
          </geometry>
          <material>
            <ambient>%(color)s</ambient>
            <diffuse>%(color)s</diffuse>
          </material>
        </visual>
      </link>
    </model>

"""


def _rect_to_box(rect):
    """(xmin, ymin, xmax, ymax, h) -> centered box size + pose + name."""
    xmin, ymin, xmax, ymax, h = rect
    cx = (xmin + xmax) / 2.0
    cy = (ymin + ymax) / 2.0
    sx = xmax - xmin
    sy = ymax - ymin
    return cx, cy, sx, sy, h


def build_models():
    """Return the SDF model block for every wall and obstacle."""
    blocks = []
    for i, rect in enumerate(WALLS):
        cx, cy, sx, sy, h = _rect_to_box(rect)
        blocks.append(
            _MODEL_TEMPLATE
            % {
                "name": "wall_%02d" % i,
                "cx": cx,
                "cy": cy,
                "sx": sx,
                "sy": sy,
                "h": h,
                "hz": h / 2.0,
                "color": "0.55 0.55 0.58 1",
            }
        )
    for i, rect in enumerate(OBSTACLES):
        cx, cy, sx, sy, h = _rect_to_box(rect)
        blocks.append(
            _MODEL_TEMPLATE
            % {
                "name": "obstacle_%02d" % i,
                "cx": cx,
                "cy": cy,
                "sx": sx,
                "sy": sy,
                "h": h,
                "hz": h / 2.0,
                "color": "0.35 0.45 0.65 1",
            }
        )
    return "".join(blocks)


def build_sdf():
    return _SDF_TEMPLATE % {"models": build_models()}


# ---------------------------------------------------------------------------
# Occupancy map
# ---------------------------------------------------------------------------

def build_pgm():
    """Occupancy grid: 0 = occupied, 255 = free (negate: 0)."""
    width = int((WORLD_X + 2 * MARGIN) / RES)
    height = int((WORLD_Y + 2 * MARGIN) / RES)
    grid = [[255] * width for _ in range(height)]

    def mark(rect):
        xmin, ymin, xmax, ymax, _h = rect
        px0 = max(0, int((xmin + MARGIN) / RES))
        px1 = min(width - 1, int((xmax + MARGIN) / RES))
        # Image row 0 = NORTH edge (map_server convention: the yaml origin
        # is the pose of the lower-left pixel, so world y grows downward in
        # the file). Without the flip the map is vertically mirrored and
        # AMCL can never match the world.
        py0 = max(0, int((WORLD_Y + MARGIN - ymax) / RES))
        py1 = min(height - 1, int((WORLD_Y + MARGIN - ymin) / RES))
        for py in range(py0, py1 + 1):
            for px in range(px0, px1 + 1):
                grid[py][px] = 0

    for rect in WALLS + OBSTACLES:
        mark(rect)

    # P5 binary PGM.
    body = b"".join(bytes(row) for row in grid)
    return (
        "P5\n%d %d\n255\n" % (width, height)
    ).encode("ascii") + body


def build_map_yaml():
    width = int((WORLD_X + 2 * MARGIN) / RES)
    height = int((WORLD_Y + 2 * MARGIN) / RES)
    return (
        "image: amr_office.pgm\n"
        "resolution: %g\n"
        "origin: [%g, %g, 0.0]\n"
        "negate: 0\n"
        "occupied_thresh: 0.65\n"
        "free_thresh: 0.196\n"
        "mode: trinary\n"
        "# Generated by amr_simulation/tools/generate_office_world.py\n"
        "# (%dx%d px, %g m/px, covers world [-%g, %g] x [-%g, %g])\n"
        % (
            RES,
            -MARGIN,
            -MARGIN,
            width,
            height,
            RES,
            MARGIN,
            WORLD_X + MARGIN,
            MARGIN,
            WORLD_Y + MARGIN,
        )
    )


def main(argv=None):
    """Generate the world + map. `argv` overrides sys.argv (tests use it)."""
    here = os.path.dirname(os.path.abspath(__file__))
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--sdf-out",
        default=os.path.normpath(os.path.join(here, "..", "worlds")),
        help="Directory for amr_office.sdf (default: amr_simulation/worlds)",
    )
    parser.add_argument(
        "--map-out",
        default=os.path.normpath(
            os.path.join(here, "..", "..", "amr_navigation", "maps")
        ),
        help="Directory for amr_office.pgm/.yaml (default: amr_navigation/maps)",
    )
    args = parser.parse_args(argv)

    sdf_path = os.path.join(args.sdf_out, "amr_office.sdf")
    pgm_path = os.path.join(args.map_out, "amr_office.pgm")
    yaml_path = os.path.join(args.map_out, "amr_office.yaml")

    with open(sdf_path, "w") as f:
        f.write(build_sdf())
    with open(pgm_path, "wb") as f:
        f.write(build_pgm())
    with open(yaml_path, "w") as f:
        f.write(build_map_yaml())

    width = int((WORLD_X + 2 * MARGIN) / RES)
    height = int((WORLD_Y + 2 * MARGIN) / RES)
    print(
        "wrote %s (%dx%d px)\n     %s\n     %s\nspawn/AMCL initial: %s"
        % (sdf_path, width, height, pgm_path, yaml_path, SPAWN_POSE)
    )


if __name__ == "__main__":
    sys.exit(main())
