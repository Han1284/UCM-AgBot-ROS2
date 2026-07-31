#!/usr/bin/env python3
"""Generate the Pro450-only half-scale plant SDF from the shared source."""

import argparse
from pathlib import Path
import xml.etree.ElementTree as ET


SCALE = 0.5


def scale_vector(element, component_count):
    values = [float(value) for value in element.text.split()]
    values[:component_count] = [
        value * SCALE for value in values[:component_count]
    ]
    element.text = ' '.join(f'{value:.10g}' for value in values)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('source', type=Path)
    parser.add_argument('destination', type=Path)
    args = parser.parse_args()

    tree = ET.parse(args.source)
    root = tree.getroot()
    model = root.find('model')
    model.set('name', 'simple_potted_plant_pro450')

    for pose in model.findall('.//visual/pose'):
        scale_vector(pose, 3)
    for mesh_scale in model.findall('.//visual/geometry/mesh/scale'):
        scale_vector(mesh_scale, 3)
    for pose in model.findall('.//collision/pose'):
        scale_vector(pose, 3)
    for box_size in model.findall('.//collision/geometry/box/size'):
        scale_vector(box_size, 3)
    for cylinder in model.findall('.//collision/geometry/cylinder'):
        cylinder.find('radius').text = (
            f'{float(cylinder.findtext("radius")) * SCALE:.10g}')
        cylinder.find('length').text = (
            f'{float(cylinder.findtext("length")) * SCALE:.10g}')

    args.destination.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(tree, space='  ')
    tree.write(args.destination, encoding='unicode', xml_declaration=True)


if __name__ == '__main__':
    main()
