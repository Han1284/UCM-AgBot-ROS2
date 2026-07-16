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
        # The official D435 Collada mesh is useful in RViz but can stall Ogre
        # while a depth sensor renders in a VM. Gazebo only needs a light body.
        if link.get('name') == 'camera_link':
            for visual in list(link.findall('visual')):
                geometry = visual.find('geometry')
                mesh = geometry.find('mesh') if geometry is not None else None
                if mesh is not None and mesh.get('filename', '').endswith('d435.dae'):
                    link.remove(visual)
                    safe_visual = ET.Element('visual', name='gazebo_d435_body')
                    ET.SubElement(safe_visual, 'origin', xyz='0 -0.0175 0', rpy='0 0 0')
                    safe_geometry = ET.SubElement(safe_visual, 'geometry')
                    ET.SubElement(safe_geometry, 'box', size='0.02505 0.090 0.025')
                    material = ET.SubElement(safe_visual, 'material', name='camera_gray')
                    ET.SubElement(material, 'color', rgba='0.55 0.57 0.59 1')
                    link.append(safe_visual)

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
    tree.write(sys.stdout, encoding='unicode', xml_declaration=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
