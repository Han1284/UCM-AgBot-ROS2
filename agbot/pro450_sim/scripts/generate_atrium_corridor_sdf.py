#!/usr/bin/env python3
"""Generate Fortress SDF for a 10x10 m 回-shaped indoor corridor + 6x6 atrium."""

from __future__ import annotations

import argparse
from pathlib import Path

# Corridor outer half-extent, atrium half-extent → corridor width = 2 m
OUTER = 5.0
ATRIUM = 3.0
WALL_T = 0.12
WALL_H = 3.0
FLOOR_T = 0.08
# Symmetric doors on outer and atrium walls (one centered per side)
DOOR_W = 1.2
DOOR_H = 2.2
DOOR_T = 0.045
FRAME_T = 0.08
FRAME_DEPTH = 0.16
LEAF_GAP = 0.02  # clearance inside frame

FLOORS = {
    # name: (albedo filename under materials/textures, diffuse RGBA fallback)
    'wood': ('wood_floor.jpg', '0.62 0.48 0.34 1'),
    'cherry': ('wood_cherry.png', '0.58 0.38 0.28 1'),
    'concrete': ('concrete.png', '0.58 0.58 0.56 1'),
    'checker': ('checker.png', '0.92 0.92 0.92 1'),
    'plywood': ('plywood.png', '0.72 0.62 0.48 1'),
    'tarmac': ('tarmac.png', '0.28 0.28 0.28 1'),
    'plain': (None, '0.78 0.78 0.76 1'),
}

ATRIUM_FLOORS = {
    'stone': '0.72 0.72 0.70 1',
    'dark': '0.35 0.38 0.36 1',
    'green': '0.45 0.55 0.40 1',
}


def box(name: str, size_xyz: str, pose: str, material: str, coll: bool = True) -> str:
    collision = ''
    if coll:
        collision = f'''
      <collision name="{name}_col">
        <geometry><box><size>{size_xyz}</size></box></geometry>
      </collision>'''
    return f'''
    <link name="{name}">
      <pose>{pose}</pose>
      <inertial><mass>1</mass><inertia><ixx>1</ixx><iyy>1</iyy><izz>1</izz></inertia></inertial>{collision}
      <visual name="{name}_vis">
        <geometry><box><size>{size_xyz}</size></box></geometry>
        {material}
      </visual>
    </link>'''


def mat_solid(rgba: str, specular: str = '0.08 0.08 0.08 1') -> str:
    return f'''<material>
          <ambient>{rgba}</ambient>
          <diffuse>{rgba}</diffuse>
          <specular>{specular}</specular>
        </material>'''


def mat_textured(rgba: str, albedo: str | None, model_uri_base: str) -> str:
    if not albedo:
        return mat_solid(rgba)
    uri = f'{model_uri_base}/materials/textures/{albedo}'
    return f'''<material>
          <ambient>{rgba}</ambient>
          <diffuse>{rgba}</diffuse>
          <specular>0.12 0.12 0.12 1</specular>
          <pbr>
            <metal>
              <albedo_map>{uri}</albedo_map>
              <metalness>0.0</metalness>
              <roughness>0.85</roughness>
            </metal>
          </pbr>
        </material>'''


def build_model_sdf(floor: str, atrium_style: str, use_file_uri: bool, model_dir: Path) -> str:
    albedo, floor_rgba = FLOORS[floor]
    atrium_rgba = ATRIUM_FLOORS[atrium_style]
    if use_file_uri:
        base = 'file://' + str(model_dir.resolve())
    else:
        base = 'model://atrium_corridor_10x10'

    floor_mat = mat_textured(floor_rgba, albedo, base)
    atrium_mat = mat_solid(atrium_rgba, '0.15 0.15 0.15 1')
    wall_mat = mat_solid('0.88 0.88 0.86 1', '0.05 0.05 0.05 1')
    trim_mat = mat_solid('0.40 0.42 0.45 1')
    frame_mat = mat_solid('0.32 0.28 0.24 1', '0.12 0.12 0.12 1')
    # Door leaf: wood albedo if present, else solid
    leaf_mat = mat_textured('0.48 0.34 0.22 1', 'wood_cherry.png', base)
    handle_mat = mat_solid('0.75 0.72 0.68 1', '0.4 0.4 0.4 1')

    leaf_w = DOOR_W - 2 * LEAF_GAP
    leaf_h = DOOR_H - LEAF_GAP

    # Floor ring: N/S full 10x2, E/W only 6x2 (corners owned by N/S)
    ring_w = OUTER - ATRIUM  # 2.0
    ns_y = (OUTER + ATRIUM) / 2.0  # 4.0
    ew_x = (OUTER + ATRIUM) / 2.0  # 4.0
    links = []

    # Corridor floors (top at z=0)
    z_floor = -FLOOR_T / 2.0
    links.append(box('floor_n', f'{2 * OUTER} {ring_w} {FLOOR_T}',
                     f'0 {ns_y} {z_floor} 0 0 0', floor_mat))
    links.append(box('floor_s', f'{2 * OUTER} {ring_w} {FLOOR_T}',
                     f'0 {-ns_y} {z_floor} 0 0 0', floor_mat))
    links.append(box('floor_e', f'{ring_w} {2 * ATRIUM} {FLOOR_T}',
                     f'{ew_x} 0 {z_floor} 0 0 0', floor_mat))
    links.append(box('floor_w', f'{ring_w} {2 * ATRIUM} {FLOOR_T}',
                     f'{-ew_x} 0 {z_floor} 0 0 0', floor_mat))

    # Atrium courtyard floor (slightly recessed)
    z_atrium = -FLOOR_T / 2.0 - 0.02
    links.append(box('floor_atrium', f'{2 * ATRIUM} {2 * ATRIUM} {FLOOR_T}',
                     f'0 0 {z_atrium} 0 0 0', atrium_mat))

    # Outer walls with centered door openings + closed door leaves (no roof)
    z_wall = WALL_H / 2.0
    L = 2 * OUTER
    seg = (L - DOOR_W) / 2.0  # wall segment each side of door
    seg_c = DOOR_W / 2.0 + seg / 2.0
    z_door = DOOR_H / 2.0
    z_leaf = leaf_h / 2.0
    z_lintel = DOOR_H + (WALL_H - DOOR_H) / 2.0
    lintel_h = WALL_H - DOOR_H

    # N/S walls (along X): left/right segments + lintel + frame + leaf
    for side, y in (('n', OUTER), ('s', -OUTER)):
        links.append(box(f'wall_{side}_l', f'{seg} {WALL_T} {WALL_H}',
                         f'{-seg_c} {y} {z_wall} 0 0 0', wall_mat))
        links.append(box(f'wall_{side}_r', f'{seg} {WALL_T} {WALL_H}',
                         f'{seg_c} {y} {z_wall} 0 0 0', wall_mat))
        links.append(box(f'wall_{side}_lintel', f'{DOOR_W} {WALL_T} {lintel_h}',
                         f'0 {y} {z_lintel} 0 0 0', wall_mat))
        links.append(box(f'door_{side}_jamb_l', f'{FRAME_T} {FRAME_DEPTH} {DOOR_H}',
                         f'{-DOOR_W / 2.0} {y} {z_door} 0 0 0', frame_mat))
        links.append(box(f'door_{side}_jamb_r', f'{FRAME_T} {FRAME_DEPTH} {DOOR_H}',
                         f'{DOOR_W / 2.0} {y} {z_door} 0 0 0', frame_mat))
        links.append(box(f'door_{side}_head', f'{DOOR_W + FRAME_T} {FRAME_DEPTH} {FRAME_T}',
                         f'0 {y} {DOOR_H} 0 0 0', frame_mat))
        # Closed door leaf in the opening
        links.append(box(f'door_{side}_leaf', f'{leaf_w} {DOOR_T} {leaf_h}',
                         f'0 {y} {z_leaf} 0 0 0', leaf_mat))
        # Handle on corridor side
        hy = y - (0.04 if y > 0 else -0.04)
        links.append(box(f'door_{side}_handle', f'0.02 0.06 0.12',
                         f'{leaf_w * 0.35} {hy} 1.0 0 0 0', handle_mat))

    # E/W walls (along Y): segments + lintel + frame + leaf
    ew_span = L - WALL_T
    ew_seg = (ew_span - DOOR_W) / 2.0
    ew_seg_c = DOOR_W / 2.0 + ew_seg / 2.0
    for side, x in (('e', OUTER), ('w', -OUTER)):
        links.append(box(f'wall_{side}_a', f'{WALL_T} {ew_seg} {WALL_H}',
                         f'{x} {ew_seg_c} {z_wall} 0 0 0', wall_mat))
        links.append(box(f'wall_{side}_b', f'{WALL_T} {ew_seg} {WALL_H}',
                         f'{x} {-ew_seg_c} {z_wall} 0 0 0', wall_mat))
        links.append(box(f'wall_{side}_lintel', f'{WALL_T} {DOOR_W} {lintel_h}',
                         f'{x} 0 {z_lintel} 0 0 0', wall_mat))
        links.append(box(f'door_{side}_jamb_a', f'{FRAME_DEPTH} {FRAME_T} {DOOR_H}',
                         f'{x} {DOOR_W / 2.0} {z_door} 0 0 0', frame_mat))
        links.append(box(f'door_{side}_jamb_b', f'{FRAME_DEPTH} {FRAME_T} {DOOR_H}',
                         f'{x} {-DOOR_W / 2.0} {z_door} 0 0 0', frame_mat))
        links.append(box(f'door_{side}_head', f'{FRAME_DEPTH} {DOOR_W + FRAME_T} {FRAME_T}',
                         f'{x} 0 {DOOR_H} 0 0 0', frame_mat))
        links.append(box(f'door_{side}_leaf', f'{DOOR_T} {leaf_w} {leaf_h}',
                         f'{x} 0 {z_leaf} 0 0 0', leaf_mat))
        hx = x - (0.04 if x > 0 else -0.04)
        links.append(box(f'door_{side}_handle', f'0.06 0.02 0.12',
                         f'{hx} {leaf_w * 0.35} 1.0 0 0 0', handle_mat))

    # Atrium: solid full-height walls, no doors
    Al = 2 * ATRIUM
    links.append(box('atrium_wall_n', f'{Al + WALL_T} {WALL_T} {WALL_H}',
                     f'0 {ATRIUM} {z_wall} 0 0 0', wall_mat))
    links.append(box('atrium_wall_s', f'{Al + WALL_T} {WALL_T} {WALL_H}',
                     f'0 {-ATRIUM} {z_wall} 0 0 0', wall_mat))
    links.append(box('atrium_wall_e', f'{WALL_T} {Al - WALL_T} {WALL_H}',
                     f'{ATRIUM} 0 {z_wall} 0 0 0', wall_mat))
    links.append(box('atrium_wall_w', f'{WALL_T} {Al - WALL_T} {WALL_H}',
                     f'{-ATRIUM} 0 {z_wall} 0 0 0', wall_mat))

    # Corner posts at atrium corners
    post = 0.18
    for i, (x, y) in enumerate((
        (ATRIUM, ATRIUM), (ATRIUM, -ATRIUM), (-ATRIUM, ATRIUM), (-ATRIUM, -ATRIUM)
    )):
        links.append(box(f'post_{i}', f'{post} {post} {WALL_H}',
                         f'{x} {y} {z_wall} 0 0 0', trim_mat))

    return f'''<?xml version="1.0"?>
<sdf version="1.8">
  <model name="atrium_corridor_10x10">
    <static>true</static>
{''.join(links)}
  </model>
</sdf>
'''


def build_world_sdf(floor: str, atrium_style: str, model_dir: Path, plant_uri: str) -> str:
    # World inlines the model via file:// so launch does not depend on resource path alone.
    model_uri = 'file://' + str((model_dir / 'model.sdf').resolve())

    # 8 pots: 2 per outer wall, inside corridor, clear of centered doors.
    # Corridor band is |coord| in (3, 5); sit ~0.35 m off the outer wall.
    inset = OUTER - 0.35  # 4.65
    along = 2.5
    plant_poses = [
        (along, inset, 0.0, 'plant_n_e'),
        (-along, inset, 0.0, 'plant_n_w'),
        (along, -inset, 0.0, 'plant_s_e'),
        (-along, -inset, 0.0, 'plant_s_w'),
        (inset, along, 0.0, 'plant_e_n'),
        (inset, -along, 0.0, 'plant_e_s'),
        (-inset, along, 0.0, 'plant_w_n'),
        (-inset, -along, 0.0, 'plant_w_s'),
    ]
    plant_includes = []
    for x, y, yaw, name in plant_poses:
        plant_includes.append(f'''    <include>
      <uri>{plant_uri}</uri>
      <name>{name}</name>
      <pose>{x:.3f} {y:.3f} 0 0 0 {yaw:.6f}</pose>
    </include>''')
    plants_xml = '\n'.join(plant_includes)

    return f'''<?xml version="1.0"?>
<!-- Indoor 回 corridor: 10x10 m outer, 6x6 m atrium, 2 m walkway.
     No roof. Outer N/S/E/W doors with closed leaves; atrium solid walls.
     Eight simple_potted_plant_pro450 along outer walls.
     floor={floor}  atrium={atrium_style} -->
<sdf version="1.8">
  <world name="atrium_corridor">
    <plugin filename="ignition-gazebo-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="ignition-gazebo-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="ignition-gazebo-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="ignition-gazebo-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>

    <physics name="default_physics" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <gravity>0 0 -9.81</gravity>

    <scene>
      <ambient>0.55 0.56 0.58 1</ambient>
      <background>0.62 0.72 0.82 1</background>
      <shadows>true</shadows>
    </scene>

    <!-- Daylight through open atrium -->
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 12 0 0 0</pose>
      <diffuse>0.95 0.93 0.88 1</diffuse>
      <specular>0.35 0.35 0.32 1</specular>
      <direction>-0.25 0.15 -1</direction>
    </light>

    <!-- Soft corridor fill lights -->
    <light type="point" name="lamp_n">
      <pose>0 4 2.7 0 0 0</pose>
      <diffuse>0.7 0.7 0.65 1</diffuse>
      <specular>0.1 0.1 0.1 1</specular>
      <attenuation><range>8</range><linear>0.05</linear><quadratic>0.02</quadratic></attenuation>
    </light>
    <light type="point" name="lamp_s">
      <pose>0 -4 2.7 0 0 0</pose>
      <diffuse>0.7 0.7 0.65 1</diffuse>
      <specular>0.1 0.1 0.1 1</specular>
      <attenuation><range>8</range><linear>0.05</linear><quadratic>0.02</quadratic></attenuation>
    </light>
    <light type="point" name="lamp_e">
      <pose>4 0 2.7 0 0 0</pose>
      <diffuse>0.7 0.7 0.65 1</diffuse>
      <specular>0.1 0.1 0.1 1</specular>
      <attenuation><range>8</range><linear>0.05</linear><quadratic>0.02</quadratic></attenuation>
    </light>
    <light type="point" name="lamp_w">
      <pose>-4 0 2.7 0 0 0</pose>
      <diffuse>0.7 0.7 0.65 1</diffuse>
      <specular>0.1 0.1 0.1 1</specular>
      <attenuation><range>8</range><linear>0.05</linear><quadratic>0.02</quadratic></attenuation>
    </light>

    <include>
      <uri>{model_uri}</uri>
      <name>atrium_corridor_10x10</name>
      <pose>0 0 0 0 0 0</pose>
    </include>

{plants_xml}
  </world>
</sdf>
'''


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--floor', choices=sorted(FLOORS), default='concrete')
    parser.add_argument('--atrium', choices=sorted(ATRIUM_FLOORS), default='stone')
    parser.add_argument('--model-dir', type=Path, required=True)
    parser.add_argument('--world-out', type=Path, required=True)
    parser.add_argument(
        '--plant-uri',
        default='model://simple_potted_plant_pro450',
        help='Include URI for the potted plant model',
    )
    args = parser.parse_args()

    args.model_dir.mkdir(parents=True, exist_ok=True)
    model_sdf = build_model_sdf(args.floor, args.atrium, use_file_uri=True, model_dir=args.model_dir)
    (args.model_dir / 'model.sdf').write_text(model_sdf)
    args.world_out.parent.mkdir(parents=True, exist_ok=True)
    args.world_out.write_text(
        build_world_sdf(args.floor, args.atrium, args.model_dir, args.plant_uri))
    print(f'Wrote {args.model_dir / "model.sdf"}')
    print(f'Wrote {args.world_out}')
    print(f'floor={args.floor} atrium={args.atrium} plant={args.plant_uri}')


if __name__ == '__main__':
    main()
