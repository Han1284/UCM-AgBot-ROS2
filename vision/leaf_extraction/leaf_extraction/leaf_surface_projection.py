"""Shared same-leaf projection used by perception, RViz and MTC."""

from dataclasses import dataclass
from collections import defaultdict
import math
import os
from statistics import median
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, Pose, Quaternion
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive


PLANT_MODEL_NAME = 'plant_in_front_of_arm'


def _quaternion_from_rpy(roll, pitch, yaw):
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


def _quaternion_multiply(first, second):
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
    rotated = _quaternion_multiply(
        _quaternion_multiply(rotation, vector),
        conjugate,
    )
    return rotated.x, rotated.y, rotated.z


def _parse_pose(element):
    values = [float(value) for value in element.text.split()]
    return values[:3], _quaternion_from_rpy(*values[3:])


def _plant_model_pose(package_share):
    world_package = os.environ.get(
        'LEAF_PLANT_WORLD_PACKAGE', 'leaf_manipulation_sim')
    world_relative_path = os.environ.get(
        'LEAF_PLANT_WORLD_RELATIVE_PATH',
        os.path.join('worlds', 'leaf_bench.world'),
    )
    world_share = get_package_share_directory(world_package)
    world_root = ET.parse(
        os.path.join(world_share, world_relative_path)).getroot()
    for include in world_root.findall('.//include'):
        if include.findtext('name') == PLANT_MODEL_NAME:
            position, rotation = _parse_pose(include.find('pose'))
            scale_element = include.find('scale')
            scale = (
                tuple(float(value) for value in scale_element.text.split())
                if scale_element is not None else (1.0, 1.0, 1.0)
            )
            scale_override = os.environ.get('LEAF_PLANT_PROXY_SCALE')
            if scale_override:
                values = tuple(
                    float(value) for value in scale_override.split())
                scale = values if len(values) == 3 else (values[0],) * 3
            return position, rotation, scale
    raise RuntimeError(
        f'Plant include {PLANT_MODEL_NAME!r} was not found')


def plant_collision_objects():
    """Load proxies without importing the dependent simulation package."""
    package_share = get_package_share_directory('leaf_manipulation_sim')
    model_position, model_rotation, model_scale = _plant_model_pose(
        package_share)
    model_root = ET.parse(os.path.join(
        package_share, 'models', 'simple_potted_plant', 'model.sdf'
    )).getroot()
    objects = []
    for collision in model_root.findall(
            './/link[@name="plant_link"]/collision'):
        name = collision.attrib['name']
        if name != 'pot_collision' and not name.startswith('leaf_'):
            continue
        local_position, local_rotation = _parse_pose(
            collision.find('pose'))
        local_position = tuple(
            local_position[index] * model_scale[index]
            for index in range(3)
        )
        rotated_position = rotate_point(model_rotation, local_position)
        pose = Pose(
            position=Point(
                x=model_position[0] + rotated_position[0],
                y=model_position[1] + rotated_position[1],
                z=model_position[2] + rotated_position[2],
            ),
            orientation=_quaternion_multiply(
                model_rotation, local_rotation),
        )
        primitive = SolidPrimitive()
        box = collision.find('./geometry/box/size')
        cylinder = collision.find('./geometry/cylinder')
        if box is not None:
            primitive.type = SolidPrimitive.BOX
            primitive.dimensions = [
                float(value) * model_scale[index]
                for index, value in enumerate(box.text.split())
            ]
        elif cylinder is not None:
            primitive.type = SolidPrimitive.CYLINDER
            primitive.dimensions = [0.0, 0.0]
            primitive.dimensions[
                SolidPrimitive.CYLINDER_HEIGHT
            ] = float(cylinder.findtext('length')) * model_scale[2]
            primitive.dimensions[
                SolidPrimitive.CYLINDER_RADIUS
            ] = float(cylinder.findtext('radius')) * max(
                model_scale[0], model_scale[1])
        else:
            continue
        collision_object = CollisionObject()
        collision_object.header.frame_id = os.environ.get(
            'LEAF_PLANT_FRAME', 'world')
        collision_object.id = name
        collision_object.primitives.append(primitive)
        collision_object.primitive_poses.append(pose)
        collision_object.operation = CollisionObject.ADD
        objects.append(collision_object)
    return objects


@dataclass(frozen=True)
class ProjectionInput:
    """Minimal geometry needed to project one perceived leaf point."""

    index: int
    perceived_leaf_id: int
    point: tuple
    normal: tuple
    local_leaf_width: float


@dataclass(frozen=True)
class ProjectionResult:
    """One point constrained to a broad face of an assigned leaf proxy."""

    index: int
    perceived_leaf_id: int
    collision_leaf_id: str
    point: tuple
    distance: float
    surface_gap: float
    normal_alignment: float
    tangential_distance: float


def _leaf_name(collision_object):
    return '_'.join(collision_object.id.split('_')[:2])


def leaf_collision_groups():
    """Return collision proxy segments grouped by physical leaf."""
    grouped = defaultdict(list)
    for collision_object in plant_collision_objects():
        if collision_object.id.startswith('leaf_'):
            grouped[_leaf_name(collision_object)].append(collision_object)
    return dict(grouped)


def plant_root_anchor():
    """Return the world position at the centre of the pot rim."""
    for collision_object in plant_collision_objects():
        if collision_object.id != 'pot_collision':
            continue
        pose = collision_object.primitive_poses[0]
        primitive = collision_object.primitives[0]
        half_height = (
            0.5 * primitive.dimensions[SolidPrimitive.CYLINDER_HEIGHT])
        top_offset = rotate_point(
            pose.orientation, (0.0, 0.0, half_height))
        return (
            pose.position.x + top_offset[0],
            pose.position.y + top_offset[1],
            pose.position.z + top_offset[2],
        )
    raise RuntimeError('pot_collision was not found in plant geometry')


def primitive_surface_distance(point, collision_object):
    """Distance from a world point to an oriented collision primitive."""
    pose = collision_object.primitive_poses[0]
    primitive = collision_object.primitives[0]
    relative = (
        point[0] - pose.position.x,
        point[1] - pose.position.y,
        point[2] - pose.position.z,
    )
    inverse = Quaternion(
        x=-pose.orientation.x,
        y=-pose.orientation.y,
        z=-pose.orientation.z,
        w=pose.orientation.w,
    )
    local = rotate_point(inverse, relative)
    if primitive.type == SolidPrimitive.BOX:
        outside = [
            max(abs(local[index]) - 0.5 * primitive.dimensions[index], 0.0)
            for index in range(3)
        ]
        return math.sqrt(sum(value * value for value in outside))
    if primitive.type == SolidPrimitive.CYLINDER:
        radial = math.hypot(local[0], local[1])
        radial_gap = max(
            radial - primitive.dimensions[SolidPrimitive.CYLINDER_RADIUS],
            0.0,
        )
        axial_gap = max(
            abs(local[2])
            - 0.5 * primitive.dimensions[SolidPrimitive.CYLINDER_HEIGHT],
            0.0,
        )
        return math.hypot(radial_gap, axial_gap)
    return math.sqrt(sum(value * value for value in relative))


def box_broad_face_residual(point, collision_object):
    """Distance to a finite broad face, not merely to the box volume."""
    primitive = collision_object.primitives[0]
    if primitive.type != SolidPrimitive.BOX:
        return float('inf')
    pose = collision_object.primitive_poses[0]
    inverse = Quaternion(
        x=-pose.orientation.x,
        y=-pose.orientation.y,
        z=-pose.orientation.z,
        w=pose.orientation.w,
    )
    local = rotate_point(inverse, (
        point[0] - pose.position.x,
        point[1] - pose.position.y,
        point[2] - pose.position.z,
    ))
    half_size = [
        0.5 * primitive.dimensions[index] for index in range(3)]
    outside_x = max(abs(local[0]) - half_size[0], 0.0)
    outside_y = max(abs(local[1]) - half_size[1], 0.0)
    face_gap = abs(abs(local[2]) - half_size[2])
    return math.sqrt(
        outside_x * outside_x
        + outside_y * outside_y
        + face_gap * face_gap)


def _normalized(vector):
    norm = math.sqrt(sum(value * value for value in vector))
    if norm < 1e-9:
        return None
    return tuple(value / norm for value in vector)


def _project_to_leaf(
    candidate,
    collision_leaf_id,
    objects,
    maximum_width_ratio,
    maximum_tangential_ratio,
    minimum_normal_alignment,
    minimum_face_inset_ratio,
):
    normal = _normalized(candidate.normal)
    projection_limit = maximum_width_ratio * candidate.local_leaf_width
    tangential_limit = (
        maximum_tangential_ratio * candidate.local_leaf_width)
    if (
        normal is None
        or candidate.local_leaf_width <= 0.0
        or projection_limit <= 0.0
    ):
        return None

    surface_gap = min(
        primitive_surface_distance(candidate.point, item)
        for item in objects
    )
    best = None
    for collision_object in objects:
        primitive = collision_object.primitives[0]
        if primitive.type != SolidPrimitive.BOX:
            continue
        pose = collision_object.primitive_poses[0]
        inverse = Quaternion(
            x=-pose.orientation.x,
            y=-pose.orientation.y,
            z=-pose.orientation.z,
            w=pose.orientation.w,
        )
        local_point = rotate_point(inverse, (
            candidate.point[0] - pose.position.x,
            candidate.point[1] - pose.position.y,
            candidate.point[2] - pose.position.z,
        ))
        local_normal = rotate_point(inverse, normal)
        normal_alignment = abs(local_normal[2])
        if normal_alignment < minimum_normal_alignment:
            continue

        half_size = [
            0.5 * primitive.dimensions[index] for index in range(3)]
        face_z = math.copysign(half_size[2], local_normal[2])
        travel = (face_z - local_point[2]) / local_normal[2]
        hit_x = local_point[0] + travel * local_normal[0]
        hit_y = local_point[1] + travel * local_normal[1]
        # Trex interior rule: never clamp a near-miss onto the face rim.
        # Contacts must land inside an inset of the collision-proxy face so
        # published markers stay away from the leaf edge.
        inset_x = max(
            half_size[0] * minimum_face_inset_ratio, 1e-4)
        inset_y = max(
            half_size[1] * minimum_face_inset_ratio, 1e-4)
        if abs(hit_x) > half_size[0] - inset_x:
            continue
        if abs(hit_y) > half_size[1] - inset_y:
            continue

        local_hit = (hit_x, hit_y, face_z)
        rotated_hit = rotate_point(pose.orientation, local_hit)
        world_hit = (
            pose.position.x + rotated_hit[0],
            pose.position.y + rotated_hit[1],
            pose.position.z + rotated_hit[2],
        )
        displacement = tuple(
            world_hit[index] - candidate.point[index]
            for index in range(3)
        )
        normal_distance = abs(sum(
            displacement[index] * normal[index] for index in range(3)
        ))
        distance = math.sqrt(sum(value * value for value in displacement))
        tangential_distance = math.sqrt(max(
            distance * distance - normal_distance * normal_distance,
            0.0,
        ))
        if (
            distance > projection_limit
            or tangential_distance > tangential_limit
        ):
            continue
        ranking = (distance, tangential_distance, -normal_alignment)
        if best is None or ranking < best[0]:
            best = (
                ranking,
                ProjectionResult(
                    index=candidate.index,
                    perceived_leaf_id=candidate.perceived_leaf_id,
                    collision_leaf_id=collision_leaf_id,
                    point=world_hit,
                    distance=distance,
                    surface_gap=surface_gap,
                    normal_alignment=normal_alignment,
                    tangential_distance=tangential_distance,
                ),
            )
    return None if best is None else best[1]


def project_candidate_groups(
    candidates,
    maximum_width_ratio=1.25,
    maximum_tangential_ratio=0.35,
    minimum_normal_alignment=0.35,
    minimum_face_inset_ratio=0.20,
):
    """Assign each perceived leaf to one proxy, then project all its points."""
    collision_groups = leaf_collision_groups()
    perceived_groups = defaultdict(list)
    for candidate in candidates:
        perceived_groups[candidate.perceived_leaf_id].append(candidate)

    results = {}
    assignments = {}
    for perceived_leaf_id, group in perceived_groups.items():
        ranked_assignments = []
        projected_by_leaf = {}
        for collision_leaf_id, objects in collision_groups.items():
            projected = [
                _project_to_leaf(
                    candidate,
                    collision_leaf_id,
                    objects,
                    maximum_width_ratio,
                    maximum_tangential_ratio,
                    minimum_normal_alignment,
                    minimum_face_inset_ratio,
                )
                for candidate in group
            ]
            valid = [item for item in projected if item is not None]
            if not valid:
                continue
            ranked_assignments.append((
                -len(valid),
                median(item.distance for item in valid),
                median(item.surface_gap for item in valid),
                collision_leaf_id,
            ))
            projected_by_leaf[collision_leaf_id] = valid
        if not ranked_assignments:
            continue
        ranked_assignments.sort()
        collision_leaf_id = ranked_assignments[0][3]
        assignments[perceived_leaf_id] = collision_leaf_id
        for result in projected_by_leaf[collision_leaf_id]:
            face_residual = min(
                box_broad_face_residual(result.point, item)
                for item in collision_groups[collision_leaf_id]
            )
            if face_residual <= 1e-6:
                results[result.index] = result
    return results, assignments
