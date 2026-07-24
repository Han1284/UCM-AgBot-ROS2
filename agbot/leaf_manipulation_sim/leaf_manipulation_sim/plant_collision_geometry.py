"""Build MoveIt collision objects from the Gazebo plant SDF."""

import math
import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, Pose, Quaternion
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive


PLANT_MODEL_NAME = 'plant_in_front_of_arm'


def quaternion_from_rpy(roll, pitch, yaw):
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return Quaternion(
        x=sr * cp * cy - cr * sp * sy,
        y=cr * sp * cy + sr * cp * sy,
        z=cr * cp * sy - sr * sp * cy,
        w=cr * cp * cy + sr * sp * sy,
    )


def quaternion_multiply(first, second):
    return Quaternion(
        x=first.w * second.x + first.x * second.w
        + first.y * second.z - first.z * second.y,
        y=first.w * second.y - first.x * second.z
        + first.y * second.w + first.z * second.x,
        z=first.w * second.z + first.x * second.y
        - first.y * second.x + first.z * second.w,
        w=first.w * second.w - first.x * second.x
        - first.y * second.y - first.z * second.z,
    )


def rotate_point(rotation, point):
    vector = Quaternion(x=point[0], y=point[1], z=point[2], w=0.0)
    conjugate = Quaternion(
        x=-rotation.x,
        y=-rotation.y,
        z=-rotation.z,
        w=rotation.w,
    )
    rotated = quaternion_multiply(
        quaternion_multiply(rotation, vector),
        conjugate,
    )
    return rotated.x, rotated.y, rotated.z


def parse_pose(element):
    values = [float(value) for value in element.text.split()]
    return values[:3], quaternion_from_rpy(*values[3:])


def plant_collision_objects():
    """Load the exact Gazebo pot and leaf proxy geometry into MoveIt."""
    package_share = get_package_share_directory('leaf_manipulation_sim')
    world_root = ET.parse(
        os.path.join(package_share, 'worlds', 'leaf_bench.world')).getroot()
    model_position = None
    model_rotation = None
    for include in world_root.findall('.//include'):
        if include.findtext('name') == PLANT_MODEL_NAME:
            model_position, model_rotation = parse_pose(include.find('pose'))
            break
    if model_position is None:
        raise RuntimeError(
            f'Plant include {PLANT_MODEL_NAME!r} was not found in leaf_bench.world')

    model_root = ET.parse(
        os.path.join(
            package_share,
            'models',
            'simple_potted_plant',
            'model.sdf',
        )
    ).getroot()
    objects = []
    for collision in model_root.findall(
            './/link[@name="plant_link"]/collision'):
        name = collision.attrib['name']
        if name != 'pot_collision' and not name.startswith('leaf_'):
            continue

        local_position, local_rotation = parse_pose(collision.find('pose'))
        rotated_position = rotate_point(model_rotation, local_position)
        pose = Pose(
            position=Point(
                x=model_position[0] + rotated_position[0],
                y=model_position[1] + rotated_position[1],
                z=model_position[2] + rotated_position[2],
            ),
            orientation=quaternion_multiply(
                model_rotation,
                local_rotation,
            ),
        )

        primitive = SolidPrimitive()
        box = collision.find('./geometry/box/size')
        cylinder = collision.find('./geometry/cylinder')
        if box is not None:
            primitive.type = SolidPrimitive.BOX
            primitive.dimensions = [
                float(value) for value in box.text.split()]
        elif cylinder is not None:
            primitive.type = SolidPrimitive.CYLINDER
            primitive.dimensions = [0.0, 0.0]
            primitive.dimensions[SolidPrimitive.CYLINDER_HEIGHT] = float(
                cylinder.findtext('length'))
            primitive.dimensions[SolidPrimitive.CYLINDER_RADIUS] = float(
                cylinder.findtext('radius'))
        else:
            continue

        collision_object = CollisionObject()
        collision_object.header.frame_id = 'world'
        collision_object.id = name
        collision_object.primitives.append(primitive)
        collision_object.primitive_poses.append(pose)
        collision_object.operation = CollisionObject.ADD
        objects.append(collision_object)
    return objects
