#!/usr/bin/env python3
"""Prepare the visualization URDF for stable Gazebo Classic use."""

import sys
import xml.etree.ElementTree as ET


def _tiny_box_collision(name: str = 'gazebo_safe') -> ET.Element:
    collision = ET.Element('collision', name=name)
    ET.SubElement(collision, 'origin', xyz='0 0 0', rpy='0 0 0')
    geometry = ET.SubElement(collision, 'geometry')
    ET.SubElement(geometry, 'box', size='0.001 0.001 0.001')
    return collision


def sanitize_tree(root: ET.Element) -> None:
    for link in root.findall('link'):
        removed_mesh = False
        for collision in list(link.findall('collision')):
            geometry = collision.find('geometry')
            if geometry is not None and geometry.find('mesh') is not None:
                link.remove(collision)
                removed_mesh = True
        if removed_mesh and not link.findall('collision'):
            link.append(_tiny_box_collision())


def main() -> int:
    tree = ET.parse(sys.stdin)
    sanitize_tree(tree.getroot())
    # gazebo_ros/spawn_entity.py reads the file as text and passes that Unicode
    # string to lxml.  lxml rejects Unicode input that still has an encoding
    # declaration, so emit a declaration-free XML fragment here.
    tree.write(sys.stdout, encoding='unicode', xml_declaration=False)
    return 0


if __name__ == '__main__':
    sys.exit(main())
