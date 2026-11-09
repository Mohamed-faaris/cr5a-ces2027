#!/usr/bin/env python3
"""Generate a Gazebo Classic pick/place world with a static shelf/platform and a dynamic object."""
from __future__ import annotations

import argparse
import math
from pathlib import Path

SHELVES = {
    "compact": (0.420, 0.620, 0.300, 2, 0.018),
    "standard": (0.600, 0.900, 0.400, 3, 0.018),
    "wide": (0.900, 1.200, 0.450, 4, 0.020),
    "deep": (0.700, 1.000, 0.600, 3, 0.022),
}
OBJECTS = {
    "coke": ("cylinder", (0.066, 0.066, 0.122), 0.355),
    "pringles": ("cylinder", (0.075, 0.075, 0.240), 0.215),
    "wood_box": ("box", (0.120, 0.090, 0.080), 0.120 * 0.090 * 0.080 * 650.0),
}
SHELF_ORIGIN = (0.55, 0.0, 0.0)
SHELF_YAW = math.pi / 2  # open local +Y face points toward robot base (-X)
PLATFORM_ORIGIN = (0.35, -0.35, 0.0)
PLATFORM_SIZE = (0.30, 0.24, 0.02)


def _box_geometry(size):
    return f'<geometry><box><size>{size[0]} {size[1]} {size[2]}</size></box></geometry>'


def _link_box(name, pose, size, material='Gazebo/Grey'):
    geom = _box_geometry(size)
    return f'''<collision name="{name}_collision"><pose>{pose}</pose>{geom}</collision>
      <visual name="{name}_visual"><pose>{pose}</pose>{geom}<material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>{material}</name></script></material></visual>'''


def _inertia(shape, dims, mass):
    x, y, z = dims
    if shape == "cylinder":
        radius, height = x / 2, z
        ixx = iyy = mass * (3 * radius * radius + height * height) / 12
        izz = mass * radius * radius / 2
    else:
        ixx = mass * (y*y + z*z) / 12
        iyy = mass * (x*x + z*z) / 12
        izz = mass * (x*x + y*y) / 12
    return f'<inertia><ixx>{ixx:.9f}</ixx><iyy>{iyy:.9f}</iyy><izz>{izz:.9f}</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>'


def generate_world(profile: str, object_kind: str, output: str | Path, *, level: int = 1) -> Path:
    if profile not in SHELVES:
        raise ValueError(f"unknown shelf profile: {profile}")
    if object_kind not in OBJECTS:
        raise ValueError(f"unknown object kind: {object_kind}")
    width, height, depth, levels, board = SHELVES[profile]
    if not 0 <= level < levels:
        raise ValueError(f"level must be in [0, {levels - 1}]")
    shape, dims, mass = OBJECTS[object_kind]
    bay_h = (height - board * (levels + 1)) / levels
    bay_floor = board + level * (bay_h + board)
    clearance = 0.012
    object_center_local = (0.0, 0.0, bay_floor + clearance + dims[2] / 2)
    c, s = math.cos(SHELF_YAW), math.sin(SHELF_YAW)
    object_world = (
        SHELF_ORIGIN[0] + c*object_center_local[0] - s*object_center_local[1],
        SHELF_ORIGIN[1] + s*object_center_local[0] + c*object_center_local[1],
        SHELF_ORIGIN[2] + object_center_local[2],
    )

    parts = [
        _link_box("left_side", f"{-width/2+board/2} 0 {height/2} 0 0 0", (board, depth, height)),
        _link_box("right_side", f"{width/2-board/2} 0 {height/2} 0 0 0", (board, depth, height)),
        _link_box("back_panel", f"0 {-depth/2+board/2} {height/2} 0 0 0", (width-2*board, board, height)),
    ]
    for index in range(levels + 1):
        z = board/2 + index * (bay_h + board)
        parts.append(_link_box(f"shelf_board_{index}", f"0 0 {z} 0 0 0", (width, depth, board)))
    shelf_model = f'''<model name="pick_shelf"><static>true</static><pose>{SHELF_ORIGIN[0]} {SHELF_ORIGIN[1]} {SHELF_ORIGIN[2]} 0 0 {SHELF_YAW}</pose>
      <link name="shelf_structure">{''.join(parts)}</link></model>'''

    px, py, pz = PLATFORM_ORIGIN
    sx, sy, sz = PLATFORM_SIZE
    platform_model = f'''<model name="place_platform"><static>true</static><pose>{px} {py} {pz+sz/2} 0 0 0</pose><link name="platform">
      {_link_box("top", "0 0 0 0 0 0", PLATFORM_SIZE, "Gazebo/Blue")}
    </link></model>'''

    if shape == "cylinder":
        geom = f'<geometry><cylinder><radius>{dims[0]/2}</radius><length>{dims[2]}</length></cylinder></geometry>'
    else:
        geom = _box_geometry(dims)
    object_model = f'''<model name="pick_object"><pose>{object_world[0]} {object_world[1]} {object_world[2]} 0 0 {SHELF_YAW}</pose><link name="object">
      <inertial><mass>{mass:.6f}</mass>{_inertia(shape, dims, mass)}</inertial>
      <collision name="object_collision">{geom}<surface><friction><ode><mu>0.8</mu><mu2>0.8</mu2></ode></friction></surface></collision>
      <visual name="object_visual">{geom}<material><ambient>0.85 0.12 0.12 1</ambient><diffuse>0.85 0.12 0.12 1</diffuse></material></visual>
    </link></model>'''

    sdf = f'''<?xml version="1.0"?>
<sdf version="1.6"><world name="pick_place">
  <include><uri>model://ground_plane</uri></include><include><uri>model://sun</uri></include>
  <physics type="ode"><max_step_size>0.001</max_step_size><real_time_update_rate>1000</real_time_update_rate></physics>
  {shelf_model}
  {platform_model}
  {object_model}
</world></sdf>
'''
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(sdf, encoding="utf-8")
    return output


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", choices=SHELVES, default="standard")
    parser.add_argument("--object", choices=OBJECTS, default="coke")
    parser.add_argument("--level", type=int, default=1)
    parser.add_argument("--output", default="/tmp/cr5a_pick_place.world")
    args = parser.parse_args()
    print(generate_world(args.profile, args.object, args.output, level=args.level))


if __name__ == "__main__":
    main()
