#!/usr/bin/env python3
"""Split the plant OBJ into five leaf-only meshes for synthetic labels."""

from collections import defaultdict
from pathlib import Path


MODEL_DIR = Path(__file__).resolve().parent
SOURCE_OBJ = (
    MODEL_DIR.parent
    / 'simple_potted_plant'
    / 'meshes'
    / 'FlowerPot_fortress.obj'
)
MESH_DIR = MODEL_DIR / 'meshes'

COLORS = [
    (1.0, 0.0, 1.0),
    (0.0, 1.0, 1.0),
    (1.0, 1.0, 0.0),
    (0.0, 0.0, 1.0),
]


def vertex_index(token):
    return int(token.split('/', maxsplit=1)[0])


def connected_components(faces):
    vertex_faces = defaultdict(list)
    for face_index, tokens in enumerate(faces):
        for token in tokens:
            vertex_faces[vertex_index(token)].append(face_index)

    seen = set()
    components = []
    for seed in range(len(faces)):
        if seed in seen:
            continue
        stack = [seed]
        seen.add(seed)
        component = []
        while stack:
            face_index = stack.pop()
            component.append(faces[face_index])
            for token in faces[face_index]:
                for neighbour in vertex_faces[vertex_index(token)]:
                    if neighbour not in seen:
                        seen.add(neighbour)
                        stack.append(neighbour)
        components.append(component)
    return components


def load_obj():
    geometry_lines = []
    groups = defaultdict(list)
    group = None

    for raw_line in SOURCE_OBJ.read_text().splitlines():
        fields = raw_line.split()
        if not fields:
            continue
        if fields[0] in {'v', 'vt', 'vn'}:
            geometry_lines.append(raw_line)
        elif fields[0] == 'g':
            group = fields[1]
        elif fields[0] == 'f' and group is not None:
            groups[group].append(fields[1:])

    leaves = connected_components(groups['Plane.007'])
    if len(leaves) != 4:
        raise RuntimeError(
            f'Expected four connected leaf meshes, found {len(leaves)}'
        )
    return geometry_lines, leaves


def write_meshes(geometry_lines, leaves):
    MESH_DIR.mkdir(parents=True, exist_ok=True)
    for stale_mesh in MESH_DIR.glob('leaf_*.obj'):
        stale_mesh.unlink()
    for index, faces in enumerate(leaves, start=1):
        output = [
            '# Generated from FlowerPot_fortress.obj; leaf faces only.',
            f'o leaf_{index}',
            *geometry_lines,
            f'g leaf_{index}',
        ]
        output.extend('f ' + ' '.join(face) for face in faces)
        output.append('')
        (MESH_DIR / f'leaf_{index}.obj').write_text('\n'.join(output))


def write_sdf():
    visuals = []
    for index, color in enumerate(COLORS, start=1):
        rgba = f'{color[0]} {color[1]} {color[2]} 1'
        visuals.append(
            f'''      <visual name="leaf_{index}_label">
        <pose>0 0 0.0001 1.57079632679 0 0</pose>
        <cast_shadows>false</cast_shadows>
        <geometry>
          <mesh>
            <uri>model://perception_label_plant/meshes/leaf_{index}.obj</uri>
            <scale>0.10 0.10 0.10</scale>
          </mesh>
        </geometry>
        <material>
          <ambient>{rgba}</ambient>
          <diffuse>{rgba}</diffuse>
          <emissive>{rgba}</emissive>
          <specular>0 0 0 1</specular>
        </material>
      </visual>'''
        )

    sdf = f'''<?xml version="1.0"?>
<sdf version="1.8">
  <model name="perception_label_plant">
    <static>true</static>
    <link name="label_link">
{chr(10).join(visuals)}
    </link>
  </model>
</sdf>
'''
    (MODEL_DIR / 'model.sdf').write_text(sdf)

    config = '''<?xml version="1.0"?>
<model>
  <name>Perception Label Plant</name>
  <version>1.0</version>
  <sdf version="1.8">model.sdf</sdf>
  <author>
    <name>RoMu4o simulation</name>
  </author>
  <description>
    Four uniquely colored leaf meshes for synthetic labels.
  </description>
</model>
'''
    (MODEL_DIR / 'model.config').write_text(config)


def main():
    geometry_lines, leaves = load_obj()
    write_meshes(geometry_lines, leaves)
    write_sdf()
    print(f'Generated {len(leaves)} leaf label meshes in {MODEL_DIR}')


if __name__ == '__main__':
    main()
