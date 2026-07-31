#!/usr/bin/env python3

"""Publish ranked MoveIt Task Constructor solutions for the leaf pinch pose."""

from dataclasses import dataclass
from copy import deepcopy
import json
import math
import os
import signal
import subprocess
import sys
import tempfile
import threading
import time
from collections import defaultdict

from geometry_msgs.msg import (
    Point,
    Pose,
    PoseStamped,
    Quaternion,
    Vector3,
    Vector3Stamped,
)
from moveit_msgs.msg import AllowedCollisionEntry, MoveItErrorCodes
from moveit_msgs.msg import PlanningSceneComponents
from moveit_msgs.srv import (
    ApplyPlanningScene,
    GetPlanningScene,
    GetPositionIK,
    GetStateValidity,
)
from moveit.task_constructor import core, stages
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Header

from custom_interfaces.msg import LeafPoseArrays
from leaf_manipulation_sim.plant_collision_geometry import (
    plant_collision_objects,
    rotate_point,
)
from shape_msgs.msg import SolidPrimitive
import rclpy
import rclcpp


IK_STAGE_NAME = '6. Solve target pose IK and rank candidates'
APPROACH_STAGE_NAME = '7. Validate straight pre-grasp approach'
# Calibrated distance from the RG2 ``gripper`` frame to the leaf contact
# plane when the leaf lies between the inner fingertips.
ROBOT_PROFILE = os.environ.get('LEAF_MTC_ROBOT_PROFILE', 'tm5_rg2')
IS_PRO450 = ROBOT_PROFILE == 'pro450_f100'
ARM_GROUP = 'pro450_arm' if IS_PRO450 else 'tmr_arm'
GRIPPER_GROUP = 'f100_gripper' if IS_PRO450 else 'rg2_gripper'
IK_LINK = 'gripper_base' if IS_PRO450 else 'gripper'
FINGERTIP_LINKS = (
    ['gripper_left1', 'gripper_right1']
    if IS_PRO450 else ['left_inner_finger', 'right_inner_finger']
)
FINGERTIP_CONTACT_OFFSET = 0.090 if IS_PRO450 else 0.035
OPEN_GRIPPER_POSITION = 0.0 if IS_PRO450 else 0.10
CLOSED_GRIPPER_POSITION = 0.68
# Pro450 candidate orientations encode the observed leaf-surface normal as
# gripper local +X (see _quaternion_local_x and surface projection below).
# Retreat and Cartesian approach must use that same normal, not local +Y.
APPROACH_AXIS = (1.0, 0.0, 0.0) if IS_PRO450 else (0.0, 0.0, 1.0)


@dataclass
class RankedCandidate:
    """One perception-ranked grasp point and one romu4o roll angle."""

    target: PoseStamped
    contact_point: tuple
    leaf_id: int
    point_rank: int
    rotation_rank: int
    tilt_degrees: float
    roll_degrees: float
    perception_score: float
    geometric_score: float
    confidence: float
    view_count: int
    edge_margin: float
    local_leaf_width: float
    clearance: float
    longitudinal_ratio: float = 0.0
    collision_leaf_id: str = ''
    pregrasp_distance: float = 0.035
    associated_leaf: str = ''
    projection_distance: float = 0.0
    planning_cost: float = float('inf')
    global_score: float = 0.0


def quaternion_from_rpy(roll, pitch, yaw):
    """Return the same Z-Y-X quaternion convention used by the C++ demo."""
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


def gripper_goal(position):
    if IS_PRO450:
        return {'gripper_controller': position}
    return {
        'finger_joint': position,
        'left_inner_knuckle_joint': -position,
        'left_inner_finger_joint': position,
        'right_outer_knuckle_joint': -position,
        'right_inner_knuckle_joint': -position,
        'right_inner_finger_joint': position,
    }


def arm_home_goal():
    prefix = 'joint' if IS_PRO450 else 'joint_'
    return {f'{prefix}{index}': 0.0 for index in range(1, 7)}


def _primitive_surface_distance(point, collision_object):
    """Distance from a world point to an oriented collision primitive."""
    pose = collision_object.primitive_poses[0]
    primitive = collision_object.primitives[0]
    relative = (
        point[0] - pose.position.x,
        point[1] - pose.position.y,
        point[2] - pose.position.z,
    )
    orientation = pose.orientation
    inverse = Quaternion(
        x=-orientation.x,
        y=-orientation.y,
        z=-orientation.z,
        w=orientation.w,
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


def nearest_leaf_collision_ids(contact_point):
    """Associate a perceived contact with the nearest leaf proxy segments."""
    grouped = defaultdict(list)
    for collision_object in plant_collision_objects():
        if not collision_object.id.startswith('leaf_'):
            continue
        leaf_name = '_'.join(collision_object.id.split('_')[:2])
        grouped[leaf_name].append(collision_object)

    nearest_name = None
    nearest_distance = float('inf')
    for leaf_name, objects in grouped.items():
        distance = min(
            _primitive_surface_distance(contact_point, item)
            for item in objects
        )
        if distance < nearest_distance:
            nearest_name = leaf_name
            nearest_distance = distance
    return (
        nearest_name,
        [item.id for item in grouped.get(nearest_name, [])],
        nearest_distance,
    )


def _quaternion_local_x(orientation):
    """Return the gripper local X axis expressed in the world frame."""
    return (
        1.0 - 2.0 * (
            orientation.y * orientation.y
            + orientation.z * orientation.z),
        2.0 * (
            orientation.x * orientation.y
            + orientation.w * orientation.z),
        2.0 * (
            orientation.x * orientation.z
            - orientation.w * orientation.y),
    )


def _project_to_associated_leaf(
    point,
    surface_normal,
    local_leaf_width,
    maximum_width_ratio,
):
    """Project a perceived point onto the same geometric leaf proxy.

    The nearest leaf is used only for association.  Projection is then
    constrained to that leaf's box segments and follows the observed surface
    normal, so a point cannot snap sideways onto a neighbouring leaf.
    """
    grouped = defaultdict(list)
    for collision_object in plant_collision_objects():
        if not collision_object.id.startswith('leaf_'):
            continue
        leaf_name = '_'.join(collision_object.id.split('_')[:2])
        grouped[leaf_name].append(collision_object)

    nearest_name = None
    nearest_distance = float('inf')
    for leaf_name, objects in grouped.items():
        distance = min(
            _primitive_surface_distance(point, item)
            for item in objects
        )
        if distance < nearest_distance:
            nearest_name = leaf_name
            nearest_distance = distance

    projection_limit = maximum_width_ratio * local_leaf_width
    if (
        nearest_name is None
        or local_leaf_width <= 0.0
        or projection_limit <= 0.0
    ):
        return None, nearest_name, nearest_distance, projection_limit

    best = None
    lateral_tolerance = 0.10 * local_leaf_width
    for collision_object in grouped[nearest_name]:
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
            point[0] - pose.position.x,
            point[1] - pose.position.y,
            point[2] - pose.position.z,
        ))
        local_normal = rotate_point(inverse, surface_normal)
        normal_alignment = abs(local_normal[2])
        if normal_alignment < 0.35:
            continue

        half_size = [
            0.5 * primitive.dimensions[index] for index in range(3)]
        # Use the broad face whose outward normal agrees with the observed
        # normal.  This prevents a point near the proxy centre plane from
        # being projected onto the hidden back face.
        face_z = math.copysign(half_size[2], local_normal[2])
        travel = (face_z - local_point[2]) / local_normal[2]
        hit_x = local_point[0] + travel * local_normal[0]
        hit_y = local_point[1] + travel * local_normal[1]
        outside_x = max(abs(hit_x) - half_size[0], 0.0)
        outside_y = max(abs(hit_y) - half_size[1], 0.0)
        if math.hypot(outside_x, outside_y) > lateral_tolerance:
            continue

        local_hit = (
            min(max(hit_x, -half_size[0]), half_size[0]),
            min(max(hit_y, -half_size[1]), half_size[1]),
            face_z,
        )
        rotated_hit = rotate_point(pose.orientation, local_hit)
        world_hit = (
            pose.position.x + rotated_hit[0],
            pose.position.y + rotated_hit[1],
            pose.position.z + rotated_hit[2],
        )
        displacement = math.sqrt(sum(
            (world_hit[index] - point[index]) ** 2
            for index in range(3)
        ))
        if displacement > projection_limit:
            continue
        ranking = (displacement, -normal_alignment)
        if best is None or ranking < best[0]:
            best = (ranking, world_hit)

    return (
        None if best is None else best[1],
        nearest_name,
        nearest_distance,
        projection_limit,
    )


def project_candidates_to_leaf_surface(candidates, maximum_width_ratio):
    """Project each unique point once, then rebuild every pre-grasp pose."""
    grouped = defaultdict(list)
    for candidate in candidates:
        grouped[candidate.point_rank].append(candidate)

    projected_candidates = []
    for point_rank, point_candidates in grouped.items():
        reference = min(
            point_candidates,
            key=lambda item: (
                abs(item.tilt_degrees),
                abs(item.roll_degrees),
            ),
        )
        surface_normal = _quaternion_local_x(
            reference.target.pose.orientation)
        projected, leaf_name, surface_gap, projection_limit = (
            _project_to_associated_leaf(
                reference.contact_point,
                surface_normal,
                reference.local_leaf_width,
                maximum_width_ratio,
            ))
        if projected is None:
            print(
                '[leaf_mtc_pinch_demo] 拒绝无法安全投影的落点: '
                f'point={point_rank + 1}, nearest={leaf_name}, '
                f'surface_gap={surface_gap:.3f} m, '
                f'local_width={reference.local_leaf_width:.3f} m, '
                f'dynamic_limit={projection_limit:.3f} m；'
                '沿感知法向未命中同一叶片的有效表面。',
                flush=True,
            )
            continue

        displacement = math.sqrt(sum(
            (projected[index] - reference.contact_point[index]) ** 2
            for index in range(3)
        ))
        print(
            '[leaf_mtc_pinch_demo] 叶面约束投影: '
            f'point={point_rank + 1}, leaf={leaf_name}, '
            f'move={displacement:.3f} m / '
            f'{projection_limit:.3f} m, '
            f'xyz=({projected[0]:.4f}, {projected[1]:.4f}, '
            f'{projected[2]:.4f})',
            flush=True,
        )
        for candidate in point_candidates:
            contact_pose = Pose()
            contact_pose.position.x = projected[0]
            contact_pose.position.y = projected[1]
            contact_pose.position.z = projected[2]
            contact_pose.orientation = candidate.target.pose.orientation
            candidate.contact_point = projected
            candidate.target.pose = _retreat_to_pregrasp(contact_pose)
            candidate.associated_leaf = leaf_name
            candidate.projection_distance = displacement
            projected_candidates.append(candidate)
    return projected_candidates


def _metadata(message, field, index, default):
    values = getattr(message, field)
    return values[index] if index < len(values) else default


def _retreat_to_pregrasp(pose, distance=0.035):
    """Move the virtual gripper point backward along the configured approach."""
    result = Pose()
    result.position.x = pose.position.x
    result.position.y = pose.position.y
    result.position.z = pose.position.z
    result.orientation = pose.orientation
    tool_axis = rotate_point(result.orientation, APPROACH_AXIS)
    result.position.x -= distance * tool_axis[0]
    result.position.y -= distance * tool_axis[1]
    result.position.z -= distance * tool_axis[2]
    return result


def _dynamic_pregrasp_distance(local_leaf_width):
    """Scale the in-plane retreat to leaf width within safe robot bounds."""
    if IS_PRO450:
        # The F100 fingertips and the segmented leaf proxy are both thicker
        # than the legacy RG2 setup.  Keep the final contact pose unchanged,
        # but start the Cartesian approach far enough away that ComputeIK sees
        # a genuinely collision-free pre-grasp state.
        return min(0.075, max(0.055, 0.75 * local_leaf_width))
    return min(0.045, max(0.025, 0.50 * local_leaf_width))


def wait_for_ranked_candidates():
    """Wait for multi-view perception and expand reachability candidates."""
    rclpy.init()
    node = rclpy.create_node('leaf_mtc_target_waiter')
    received = []
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

    def callback(message):
        received.append(message)

    subscription = node.create_subscription(
        LeafPoseArrays,
        '/target_leaves_multi_pose',
        callback,
        qos,
    )
    print(
        '[leaf_mtc_pinch_demo] 等待多视角感知发布排序后的落点与姿态……',
        flush=True,
    )
    try:
        while rclpy.ok() and not received:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    del subscription
    node.destroy_node()
    if not received:
        return None

    message = received[-1]
    candidates = []
    if message.candidate_poses:
        expanded = zip(
            message.candidate_poses,
            message.candidate_point_indices,
            message.candidate_tilt_degrees,
            message.candidate_roll_degrees,
        )
    else:
        pose_groups = (
            message.poses1,
            message.poses2,
            message.poses3,
            message.poses4,
            message.poses5,
        )
        expanded = (
            (pose, point_index, 0.0, -45.0 * rotation_index)
            for rotation_index, poses in enumerate(pose_groups)
            for point_index, pose in enumerate(poses)
        )

    for pose, point_index, tilt_degrees, roll_degrees in expanded:
        score = float(_metadata(
            message, 'scores', point_index, 1.0 / (point_index + 1)))
        local_leaf_width = float(_metadata(
            message, 'local_leaf_widths', point_index, 0.0))
        pregrasp_distance = _dynamic_pregrasp_distance(
            local_leaf_width)
        target = PoseStamped()
        target.header = message.header
        # Keep the calibrated fingertip/contact geometry separate from the
        # dynamic Cartesian approach stroke.  The generated IK target is the
        # true pre-grasp pose; MoveRelative advances only pregrasp_distance
        # and therefore finishes with the fingertips, not the gripper origin,
        # at the projected leaf surface.
        target.pose = _retreat_to_pregrasp(
            pose, FINGERTIP_CONTACT_OFFSET + pregrasp_distance)
        candidates.append(RankedCandidate(
            target=target,
            contact_point=(
                pose.position.x,
                pose.position.y,
                pose.position.z,
            ),
            leaf_id=int(_metadata(
                message, 'leaf_ids', point_index, point_index + 1)),
            point_rank=point_index,
            rotation_rank=round(abs(float(roll_degrees)) / 45.0),
            tilt_degrees=float(tilt_degrees),
            roll_degrees=float(roll_degrees),
            perception_score=score,
            geometric_score=float(_metadata(
                message, 'geometric_scores', point_index, score)),
            confidence=float(_metadata(
                message, 'confidences', point_index, 1.0)),
            view_count=int(_metadata(
                message, 'view_counts', point_index, 1)),
            edge_margin=float(_metadata(
                message, 'edge_margins', point_index, 0.0)),
            local_leaf_width=local_leaf_width,
            clearance=float(_metadata(
                message, 'approach_clearances', point_index, 0.0)),
            longitudinal_ratio=float(_metadata(
                message, 'longitudinal_ratios', point_index, 0.0)),
            projection_distance=float(_metadata(
                message, 'projection_distances', point_index, 0.0)),
            collision_leaf_id=str(_metadata(
                message, 'collision_leaf_ids', point_index, '')),
            pregrasp_distance=pregrasp_distance,
        ))

    orientation_order = {
        (0.0, 0.0): 0,
        (0.0, -90.0): 1,
        (0.0, -180.0): 2,
        (0.0, 90.0): 3,
        (-10.0, 0.0): 4,
        (10.0, 0.0): 5,
        (-20.0, 0.0): 6,
        (20.0, 0.0): 7,
        (-30.0, 0.0): 8,
        (30.0, 0.0): 9,
        (0.0, -45.0): 10,
        (0.0, -135.0): 11,
    }
    candidates.sort(
        key=lambda item: (
            orientation_order.get(
                (item.tilt_degrees, item.roll_degrees),
                20 + round(abs(item.tilt_degrees) / 10.0) * 5
                + item.rotation_rank,
            ),
            -item.perception_score,
            item.point_rank,
        ))
    return candidates


class IkServicePrechecker:
    """Run interruptible IK checks before constructing MTC."""

    def __init__(self):
        self.node = rclpy.create_node('leaf_mtc_ik_prechecker')
        self.client = self.node.create_client(GetPositionIK, '/compute_ik')
        self.validity_client = self.node.create_client(
            GetStateValidity, '/check_state_validity')
        self.get_scene_client = self.node.create_client(
            GetPlanningScene, '/get_planning_scene')
        self.apply_scene_client = self.node.create_client(
            ApplyPlanningScene, '/apply_planning_scene')
        self._original_acm = None
        self.counts = {
            'bare_success': 0,
            'bare_failure': 0,
            'bare_timeout': 0,
            'collision_success': 0,
            'collision_failure': 0,
            'collision_timeout': 0,
            'collision_deferred': 0,
        }

    def wait_for_service(self, timeout_sec=5.0):
        if not self.client.wait_for_service(timeout_sec=timeout_sec):
            return False
        if IS_PRO450:
            return all((
                self.validity_client.wait_for_service(
                    timeout_sec=timeout_sec),
                self.get_scene_client.wait_for_service(
                    timeout_sec=timeout_sec),
                self.apply_scene_client.wait_for_service(
                    timeout_sec=timeout_sec),
            ))
        return True

    def close(self):
        self.node.destroy_node()

    def set_leaf_fingertip_collisions(self, allow, timeout_sec=2.0):
        """Temporarily update only leaf-segment/fingertip ACM pairs."""
        if not IS_PRO450:
            return True
        if allow:
            request = GetPlanningScene.Request()
            request.components.components = (
                PlanningSceneComponents.ALLOWED_COLLISION_MATRIX)
            future = self.get_scene_client.call_async(request)
            rclpy.spin_until_future_complete(
                self.node, future, timeout_sec=timeout_sec)
            if not future.done() or future.result() is None:
                return False
            self._original_acm = deepcopy(
                future.result().scene.allowed_collision_matrix)
            matrix = deepcopy(self._original_acm)
            leaf_ids = [
                collision_object.id
                for collision_object in plant_collision_objects()
                if collision_object.id.startswith('leaf_')
            ]
            required_names = list(dict.fromkeys(
                list(matrix.entry_names) + leaf_ids + FINGERTIP_LINKS))
            old_names = list(matrix.entry_names)
            old_rows = {
                name: list(matrix.entry_values[index].enabled)
                for index, name in enumerate(old_names)
            }
            rows = []
            for row_name in required_names:
                row = [
                    bool(
                        old_rows.get(row_name, [])[old_names.index(column_name)]
                    )
                    if (
                        row_name in old_rows
                        and column_name in old_names
                        and old_names.index(column_name)
                        < len(old_rows[row_name])
                    )
                    else False
                    for column_name in required_names
                ]
                rows.append(AllowedCollisionEntry(enabled=row))
            matrix.entry_names = required_names
            matrix.entry_values = rows
            for leaf_id in leaf_ids:
                leaf_index = required_names.index(leaf_id)
                for fingertip in FINGERTIP_LINKS:
                    tip_index = required_names.index(fingertip)
                    matrix.entry_values[leaf_index].enabled[tip_index] = True
                    matrix.entry_values[tip_index].enabled[leaf_index] = True
        else:
            if self._original_acm is None:
                return True
            matrix = self._original_acm

        request = ApplyPlanningScene.Request()
        request.scene.is_diff = True
        request.scene.allowed_collision_matrix = matrix
        future = self.apply_scene_client.call_async(request)
        rclpy.spin_until_future_complete(
            self.node, future, timeout_sec=timeout_sec)
        success = bool(
            future.done()
            and future.result() is not None
            and future.result().success
        )
        if success and not allow:
            self._original_acm = None
        return success

    def _solve(self, candidate, avoid_collisions, timeout_sec):
        request = GetPositionIK.Request()
        ik_request = request.ik_request
        ik_request.group_name = ARM_GROUP
        ik_request.ik_link_name = IK_LINK
        ik_request.pose_stamped = candidate.target
        ik_request.robot_state.is_diff = True
        ik_request.avoid_collisions = avoid_collisions
        ik_request.timeout.sec = 0
        ik_request.timeout.nanosec = int(0.15 * 1e9)
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(
            self.node, future, timeout_sec=timeout_sec)
        if not future.done():
            self.client.remove_pending_request(future)
            return 'timeout', None
        try:
            response = future.result()
        except Exception:
            return 'failure', None
        if response.error_code.val == MoveItErrorCodes.SUCCESS:
            return 'success', response.solution
        return f'failure({response.error_code.val})', None

    def check(self, candidate, timeout_sec=0.5):
        """Require unconstrained and collision-aware IK to succeed."""
        bare_status, bare_solution = self._solve(
            candidate, False, timeout_sec)
        if bare_status == 'timeout':
            self.counts['bare_timeout'] += 1
            return False, '裸 IK 服务超时'
        if bare_status != 'success':
            self.counts['bare_failure'] += 1
            return False, f'裸 IK 无解 {bare_status}'
        self.counts['bare_success'] += 1

        # /compute_ik cannot carry a per-request AllowedCollisionMatrix. For
        # Pro450, classify the actual contact pairs returned by MoveIt and
        # admit only collision-free states or the exact target-leaf/fingertip
        # pairs that the following MTC task allows. All self, wrist, camera,
        # pot, floor and non-target-leaf collisions remain rejected here.
        if IS_PRO450 and candidate.collision_leaf_id:
            request = GetStateValidity.Request()
            request.group_name = ARM_GROUP
            request.robot_state = bare_solution
            future = self.validity_client.call_async(request)
            rclpy.spin_until_future_complete(
                self.node, future, timeout_sec=timeout_sec)
            if not future.done():
                self.validity_client.remove_pending_request(future)
                self.counts['collision_timeout'] += 1
                return False, '碰撞对检查服务超时'
            response = future.result()
            if response.valid:
                self.counts['collision_success'] += 1
                return True, bare_solution
            target_prefix = f'{candidate.collision_leaf_id}_'
            allowed = True
            for contact in response.contacts:
                bodies = (
                    contact.contact_body_1,
                    contact.contact_body_2,
                )
                leaf_body = next(
                    (body for body in bodies
                     if body.startswith(target_prefix)),
                    None,
                )
                robot_body = bodies[1] if leaf_body == bodies[0] else bodies[0]
                if leaf_body is None or robot_body not in FINGERTIP_LINKS:
                    allowed = False
                    break
            if response.contacts and allowed:
                self.counts['collision_deferred'] += 1
                return True, bare_solution
            self.counts['collision_failure'] += 1
            return False, '存在非目标叶或非指尖碰撞'

        collision_status, solution = self._solve(
            candidate, True, timeout_sec)
        if collision_status == 'timeout':
            self.counts['collision_timeout'] += 1
            return False, '碰撞 IK 服务超时'
        if collision_status != 'success':
            self.counts['collision_failure'] += 1
            return False, f'有裸 IK，但被碰撞过滤 {collision_status}'
        self.counts['collision_success'] += 1
        return True, solution


def _candidate_record(candidate):
    pose = candidate.target.pose
    return {
        'frame_id': candidate.target.header.frame_id,
        'position': [
            pose.position.x, pose.position.y, pose.position.z],
        'orientation': [
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ],
        'contact_point': list(candidate.contact_point),
        'leaf_id': candidate.leaf_id,
        'point_rank': candidate.point_rank,
        'rotation_rank': candidate.rotation_rank,
        'tilt_degrees': candidate.tilt_degrees,
        'roll_degrees': candidate.roll_degrees,
        'perception_score': candidate.perception_score,
        'geometric_score': candidate.geometric_score,
        'confidence': candidate.confidence,
        'view_count': candidate.view_count,
        'edge_margin': candidate.edge_margin,
        'local_leaf_width': candidate.local_leaf_width,
        'clearance': candidate.clearance,
        'longitudinal_ratio': candidate.longitudinal_ratio,
        'collision_leaf_id': candidate.collision_leaf_id,
        'pregrasp_distance': candidate.pregrasp_distance,
        'associated_leaf': candidate.associated_leaf,
        'projection_distance': candidate.projection_distance,
    }


def _candidate_from_record(record):
    target = PoseStamped()
    target.header.frame_id = record['frame_id']
    position = record['position']
    orientation = record['orientation']
    target.pose.position = Point(
        x=position[0], y=position[1], z=position[2])
    target.pose.orientation = Quaternion(
        x=orientation[0],
        y=orientation[1],
        z=orientation[2],
        w=orientation[3],
    )
    return RankedCandidate(
        target=target,
        contact_point=tuple(record['contact_point']),
        leaf_id=record['leaf_id'],
        point_rank=record['point_rank'],
        rotation_rank=record['rotation_rank'],
        tilt_degrees=record['tilt_degrees'],
        roll_degrees=record['roll_degrees'],
        perception_score=record['perception_score'],
        geometric_score=record['geometric_score'],
        confidence=record['confidence'],
        view_count=record['view_count'],
        edge_margin=record['edge_margin'],
        local_leaf_width=record['local_leaf_width'],
        clearance=record['clearance'],
        longitudinal_ratio=record['longitudinal_ratio'],
        collision_leaf_id=record['collision_leaf_id'],
        pregrasp_distance=record['pregrasp_distance'],
        associated_leaf=record['associated_leaf'],
        projection_distance=record['projection_distance'],
    )


def _run_fast_check_subprocess(candidate, timeout_seconds):
    """Run one MTC feasibility task in a process with a real hard deadline."""
    with tempfile.TemporaryDirectory(prefix='leaf_mtc_check_') as directory:
        input_path = os.path.join(directory, 'candidate.json')
        output_path = os.path.join(directory, 'result.json')
        with open(input_path, 'w', encoding='utf-8') as stream:
            json.dump(_candidate_record(candidate), stream)
        command = [
            sys.executable,
            os.path.abspath(__file__),
            '--fast-check-child',
            input_path,
            output_path,
            *sys.argv[1:],
        ]
        verbose = os.environ.get(
            'LEAF_MTC_FAST_CHECK_VERBOSE', '').lower() in (
                '1', 'true', 'yes')
        process = subprocess.Popen(
            command,
            stdout=None if verbose else subprocess.DEVNULL,
            stderr=None if verbose else subprocess.DEVNULL,
        )
        try:
            process.wait(timeout=timeout_seconds)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()
            return None, 'timeout'
        except KeyboardInterrupt:
            process.kill()
            process.wait()
            raise
        if process.returncode != 0 or not os.path.exists(output_path):
            return None, 'child_error'
        with open(output_path, encoding='utf-8') as stream:
            result = json.load(stream)
        return result, 'completed'


def fast_check_child(input_path, output_path):
    """Child-process entry point for one bounded MTC feasibility check."""
    with open(input_path, encoding='utf-8') as stream:
        candidate = _candidate_from_record(json.load(stream))
    rclcpp.init()
    options = rclcpp.NodeOptions()
    options.automatically_declare_parameters_from_overrides = True
    # Launch-generated parameter files are scoped to this executable's node
    # name.  Keep the child name identical so robot_description, SRDF,
    # kinematics and OMPL parameters are actually applied.
    node = rclcpp.Node('leaf_mtc_pinch_demo', options)
    # Give DDS discovery and the CurrentState subscriptions a short bounded
    # settling window.  The parent process already has these connections, but
    # every hard-timeout child starts with a fresh ROS context.
    time.sleep(0.75)
    task = make_task(
        node, candidate, ik_only=True, introspection=False)
    result = task.plan(1)
    stage_names = [
        '1. Current robot state',
        '2. Open gripper',
        '4. Collision-free approach',
        IK_STAGE_NAME,
        APPROACH_STAGE_NAME,
        '8. Close to 1 mm clearance per side',
    ]
    if not IS_PRO450:
        stage_names.insert(
            2, '3. Allow selected leaf fingertip contact')
    output = {
        'success': bool(result and task.solutions),
        'stage_counts': {
            name: _stage_solution_count(task, name)
            for name in stage_names
        },
    }
    if os.environ.get(
        'LEAF_MTC_FAST_CHECK_VERBOSE', '').lower() in (
            '1', 'true', 'yes'):
        for stage_name in stage_names:
            try:
                failures = list(task[stage_name].failures)
            except (
                AttributeError, IndexError, KeyError, RuntimeError, TypeError
            ):
                continue
            for failure in failures[:3]:
                print(
                    '[leaf_mtc_fast_check] '
                    f'{stage_name}: '
                    f'{getattr(failure, "comment", str(failure))}',
                    flush=True,
                )
    if output['success']:
        output['cost'] = float(
            getattr(task.solutions[0], 'cost', 0.0))
    with open(output_path, 'w', encoding='utf-8') as stream:
        json.dump(output, stream)
    del task
    del node
    rclcpp.shutdown()
    return 0


def make_task(node, candidate, ik_only=False, introspection=True):
    """Create IK, collision, Cartesian approach and pinch validation stages."""
    target = candidate.target
    arm_group = ARM_GROUP
    gripper_group = GRIPPER_GROUP
    task = core.Task()
    task.name = (
        'Pro450 F100 complete leaf pinch task'
        if IS_PRO450 else 'TM5-900 complete leaf pinch task')
    if not introspection:
        # Bounded feasibility checks are terminal diagnostics, not RViz tasks.
        task.enableIntrospection(False)
    task.loadRobotModel(node)

    current = stages.CurrentState('1. Current robot state')
    current.timeout = 5.0
    task.add(current)

    joint_planner = core.JointInterpolationPlanner()
    joint_planner.max_velocity_scaling_factor = 0.2
    joint_planner.max_acceleration_scaling_factor = 0.2

    open_gripper = stages.MoveTo('2. Open gripper', joint_planner)
    open_gripper.group = gripper_group
    open_gripper.setGoal(gripper_goal(OPEN_GRIPPER_POSITION))
    task.add(open_gripper)

    leaf_name, selected_leaf_collisions, association_distance = (
        nearest_leaf_collision_ids(candidate.contact_point))
    if candidate.collision_leaf_id:
        leaf_name = candidate.collision_leaf_id
        selected_leaf_collisions = [
            collision_object.id
            for collision_object in plant_collision_objects()
            if collision_object.id.startswith(f'{leaf_name}_')
        ]
    if introspection:
        print(
            '[leaf_mtc_pinch_demo] 感知目标关联到 '
            f'{leaf_name}，碰撞分段表面距离 '
            f'{association_distance:.4f} m',
            flush=True,
        )
    fingertip_links = FINGERTIP_LINKS
    allow_contact = stages.ModifyPlanningScene(
        '3. Allow selected leaf fingertip contact')
    for collision_id in selected_leaf_collisions:
        allow_contact.allowCollisions(
            collision_id,
            fingertip_links,
            True,
        )
    if not IS_PRO450:
        task.add(allow_contact)

    planner = core.PipelinePlanner(node)
    planner.planner = 'RRTConnect'
    planner.max_velocity_scaling_factor = 0.2
    planner.max_acceleration_scaling_factor = 0.2
    connect = stages.Connect(
        '4. Collision-free approach',
        [(arm_group, planner)],
    )
    connect.timeout = 2.0 if IS_PRO450 else 1.0
    task.add(connect)

    generator = stages.GeneratePose('5. Selected perceived leaf target')
    if IS_PRO450:
        # The parent temporarily applies the exact leaf-segment/fingertip ACM
        # pairs, so the generated target can use the stable CurrentState scene.
        generator.setMonitoredStage(task['1. Current robot state'])
    else:
        # Preserve the established TM5/RG2 scene flow unchanged.
        generator.setMonitoredStage(
            task['3. Allow selected leaf fingertip contact'])
    generator.pose = target

    compute_ik = stages.ComputeIK(IK_STAGE_NAME, generator)
    compute_ik.group = arm_group
    compute_ik.ik_frame = PoseStamped(
        header=Header(frame_id=IK_LINK),
        pose=Pose(orientation=Quaternion(w=1.0)),
    )
    compute_ik.max_ik_solutions = 12
    compute_ik.min_solution_distance = 0.15
    compute_ik.timeout = 0.15
    compute_ik.properties.configureInitFrom(
        core.Stage.PropertyInitializerSource.INTERFACE,
        ['target_pose'],
    )
    task.add(compute_ik)

    cartesian = core.CartesianPath()
    cartesian.max_velocity_scaling_factor = 0.12
    cartesian.max_acceleration_scaling_factor = 0.12
    # Humble requires at least ten interpolated states before its joint-jump
    # check is meaningful.  The compact Pro450 approach needs a denser step
    # than the legacy TM5/RG2 trajectory.
    cartesian.step_size = 0.0015 if IS_PRO450 else 0.003
    approach = stages.MoveRelative(
        APPROACH_STAGE_NAME,
        cartesian,
    )
    approach.group = arm_group
    approach.timeout = 0.5
    approach.ik_frame = PoseStamped(
        header=Header(frame_id=IK_LINK),
        pose=Pose(orientation=Quaternion(w=1.0)),
    )
    approach.setDirection(Vector3Stamped(
        header=Header(frame_id=IK_LINK),
        vector=Vector3(
            x=APPROACH_AXIS[0] * candidate.pregrasp_distance,
            y=APPROACH_AXIS[1] * candidate.pregrasp_distance,
            z=APPROACH_AXIS[2] * candidate.pregrasp_distance,
        ),
    ))
    task.add(approach)

    close_gripper = stages.MoveTo(
        '8. Close to 1 mm clearance per side',
        joint_planner,
    )
    close_gripper.group = gripper_group
    close_gripper.setGoal(gripper_goal(CLOSED_GRIPPER_POSITION))
    task.add(close_gripper)

    # The bounded candidate check must include closure.  A pose that reaches
    # the leaf but cannot close without collision is not a graspable landing
    # point and must never win the global ranking.
    if ik_only:
        return task

    release_gripper = stages.MoveTo('9. Re-open gripper', joint_planner)
    release_gripper.group = gripper_group
    release_gripper.setGoal(gripper_goal(OPEN_GRIPPER_POSITION))
    task.add(release_gripper)

    return_planner = core.PipelinePlanner(node)
    return_planner.planner = 'RRTConnect'
    return_planner.max_velocity_scaling_factor = 0.2
    return_planner.max_acceleration_scaling_factor = 0.2
    return_home = stages.MoveTo(
        '10. Return arm to vertical home',
        return_planner,
    )
    return_home.group = arm_group
    return_home.timeout = 1.0
    return_home.setGoal(arm_home_goal())
    task.add(return_home)

    if not IS_PRO450:
        forbid_contact = stages.ModifyPlanningScene(
            '11. Restore selected leaf collision checking')
        for collision_id in selected_leaf_collisions:
            forbid_contact.allowCollisions(
                collision_id,
                fingertip_links,
                False,
            )
        task.add(forbid_contact)
    return task


def _stage_solution_count(task, stage_name):
    """Return the number of solutions currently produced by one stage."""
    try:
        return len(task[stage_name].solutions)
    except (AttributeError, IndexError, KeyError, RuntimeError, TypeError):
        return 0


def plan_with_stage_progress(
    task,
    max_solutions,
    context,
    timeout_seconds,
):
    """Run MTC with stage progress and a hard preempting deadline."""
    finished = threading.Event()
    timed_out = threading.Event()
    start_time = time.monotonic()

    def report_progress():
        while not finished.is_set():
            elapsed = time.monotonic() - start_time
            if elapsed >= timeout_seconds:
                timed_out.set()
                print(
                    f'[leaf_mtc_pinch_demo] {context} 超过 '
                    f'{timeout_seconds:.1f} s，正在中止当前候选。',
                    flush=True,
                )
                task.preempt()
                return
            # Fast failures need no heartbeat. The first report is after 1 s.
            wait_seconds = 1.0 if elapsed < 1.0 else 2.0
            wait_seconds = min(
                wait_seconds,
                max(0.01, timeout_seconds - elapsed),
            )
            if finished.wait(wait_seconds):
                return
            elapsed = time.monotonic() - start_time
            if elapsed >= timeout_seconds:
                continue

    reporter = threading.Thread(
        target=report_progress,
        name='leaf_mtc_stage_progress',
        daemon=True,
    )
    reporter.start()
    try:
        result = task.plan(max_solutions)
        return result, timed_out.is_set()
    finally:
        finished.set()
        reporter.join(timeout=0.25)


def select_feasible_candidate(candidates, ik_prechecker):
    """Shortlist by perception, then run bounded IK/collision/line checks."""
    if not candidates:
        return None
    # The Pro 450 workspace and joint limits make valid approach poses sparser
    # than on the legacy TM5 setup.  Keep the legacy search budget unchanged,
    # but allow the Pro profile to cover the full ranked candidate set.
    default_maximum_checks = '200' if IS_PRO450 else '80'
    maximum_checks = int(
        os.environ.get('LEAF_MTC_MAX_FAST_CHECKS', default_maximum_checks)
    )
    fast_timeout = float(os.environ.get(
        'LEAF_MTC_FAST_TIMEOUT_SECONDS', '3.0'))
    projection_ratio = float(os.environ.get(
        'LEAF_MTC_MAX_PROJECTION_WIDTH_RATIO', '1.25'))
    already_projected = all(
        candidate.collision_leaf_id
        and candidate.longitudinal_ratio >= 0.30
        for candidate in candidates
    )
    if already_projected:
        print(
            '[leaf_mtc_pinch_demo] 使用感知阶段已验证的投影后落点；'
            'MTC 不再二次移动接触点。',
            flush=True,
        )
    else:
        candidates = project_candidates_to_leaf_surface(
            candidates, projection_ratio)
    if not candidates:
        print(
            '[leaf_mtc_pinch_demo] 没有落点通过叶面约束投影，'
            '未进入 IK 检查。',
            flush=True,
        )
        return None
    feasible = []
    checked_count = 0
    timeout_count = 0
    child_error_count = 0
    mtc_reject_count = 0
    mtc_failure_stages = defaultdict(int)
    best_perception = candidates[0].perception_score
    total_checks = min(len(candidates), maximum_checks)
    selection_start = time.monotonic()
    for check_index, candidate in enumerate(
        candidates[:maximum_checks], start=1
    ):
        checked_count = check_index
        percentage = 100.0 * check_index / max(total_checks, 1)
        print(
            '[leaf_mtc_pinch_demo] '
            f'[{check_index}/{total_checks}] 正在解算 '
            f'({percentage:.1f}%): '
            f'leaf={candidate.leaf_id}, point={candidate.point_rank + 1}, '
            f'tilt={candidate.tilt_degrees:+.0f} deg, '
            f'roll={candidate.roll_degrees:+.0f} deg, '
            f'perception={candidate.perception_score:.3f}',
            flush=True,
        )
        ik_valid, _ = ik_prechecker.check(candidate)
        if not ik_valid:
            continue
        result, check_status = _run_fast_check_subprocess(
            candidate, fast_timeout)
        if check_status == 'timeout':
            timeout_count += 1
            continue
        if check_status == 'child_error':
            child_error_count += 1
            continue
        if result and result.get('success'):
            raw_cost = float(result.get('cost', 0.0))
            candidate.planning_cost = max(0.0, raw_cost)
            planning_factor = 1.0 / (1.0 + 0.05 * candidate.planning_cost)
            orientation_factor = math.exp(
                -abs(candidate.tilt_degrees) / 60.0)
            candidate.global_score = (
                candidate.perception_score
                * orientation_factor
                * planning_factor
            )
            feasible.append(candidate)
            feasible_points = {
                (item.leaf_id, item.point_rank)
                for item in feasible
            }
            # Preserve some planning diversity without spending the remainder
            # of the 80-check budget after several independent landing points
            # are already fully feasible.
            if len(feasible) >= 4 and len(feasible_points) >= 2:
                break
            # A very strong top-ranked point may stop slightly earlier once
            # two of its orientations have succeeded.
            if all((
                candidate.perception_score >= best_perception - 1e-6,
                candidate.planning_cost <= 2.0,
                len(feasible) >= 2,
            )):
                break
        else:
            mtc_reject_count += 1
            stage_counts = (result or {}).get('stage_counts', {})
            for stage_name, solution_count in stage_counts.items():
                if int(solution_count) == 0:
                    mtc_failure_stages[stage_name] += 1
                    break
    total_elapsed = time.monotonic() - selection_start
    print(
        '[leaf_mtc_pinch_demo] 快速解算结束: '
        f'已检查={checked_count}/{total_checks}, '
        f'可达={len(feasible)}, '
        f'裸IK成功={ik_prechecker.counts["bare_success"]}, '
        f'碰撞IK成功={ik_prechecker.counts["collision_success"]}, '
        f'碰撞检查转交MTC={ik_prechecker.counts["collision_deferred"]}, '
        f'MTC拒绝={mtc_reject_count}, '
        f'子进程错误={child_error_count}, '
        f'超时={timeout_count}, '
        f'首个失败阶段={dict(mtc_failure_stages)}, '
        f'总耗时={total_elapsed:.2f} s。',
        flush=True,
    )
    if not feasible:
        return None
    feasible.sort(
        key=lambda item: (
            -item.global_score,
            item.planning_cost,
            item.point_rank,
            item.rotation_rank,
        ))
    return feasible[0]


def main():
    candidates = wait_for_ranked_candidates()
    if not candidates:
        print(
            '[leaf_mtc_pinch_demo] 未收到有效多视角抓取候选，规划器退出。',
            flush=True,
        )
        if rclpy.ok():
            rclpy.shutdown()
        return

    print(
        f'[leaf_mtc_pinch_demo] 收到 {len(candidates)} 个点-角度组合，'
        '开始有界快速可达性筛选。',
        flush=True,
    )
    rclcpp.init()
    node_options = rclcpp.NodeOptions()
    node_options.automatically_declare_parameters_from_overrides = True
    node = rclcpp.Node('leaf_mtc_pinch_demo', node_options)
    ik_prechecker = IkServicePrechecker()
    if not ik_prechecker.wait_for_service():
        print(
            '[leaf_mtc_pinch_demo] /compute_ik 服务不可用，'
            '无法执行有界裸 IK 与碰撞 IK 预检。',
            flush=True,
        )
        ik_prechecker.close()
        if rclpy.ok():
            rclpy.shutdown()
        del node
        rclcpp.shutdown()
        return
    ik_only = os.environ.get(
        'LEAF_MTC_IK_ONLY', 'false').lower() in ('1', 'true', 'yes')
    execute_solution = os.environ.get(
        'LEAF_MTC_EXECUTE', 'false').lower() in ('1', 'true', 'yes')
    stop_requested = [False]

    def request_stop(_signum, _frame):
        stop_requested[0] = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    task = None
    acm_enabled = False
    try:
        if IS_PRO450:
            acm_enabled = ik_prechecker.set_leaf_fingertip_collisions(True)
            if not acm_enabled:
                print(
                    '[leaf_mtc_pinch_demo] 无法应用叶片与指尖的临时碰撞'
                    '矩阵，已取消规划且未放宽其他碰撞。',
                    flush=True,
                )
                return
        selected = select_feasible_candidate(
            candidates, ik_prechecker)
        if selected is None:
            print(
                '[leaf_mtc_pinch_demo] 所有短名单候选均未通过 '
                'IK、碰撞和直线预抓取检查。',
                flush=True,
            )
            return
        target = selected.target
        print(
            '[leaf_mtc_pinch_demo] 全局最优候选: '
            f'leaf={selected.leaf_id}, point={selected.point_rank + 1}, '
            f'tilt={selected.tilt_degrees:+.0f} deg, '
            f'roll={selected.roll_degrees:+.0f} deg, '
            f'perception={selected.perception_score:.3f}, '
            f'global={selected.global_score:.3f}, '
            f'frame={target.header.frame_id}, '
            f'xyz=({target.pose.position.x:.4f}, '
            f'{target.pose.position.y:.4f}, {target.pose.position.z:.4f})',
            flush=True,
        )
        task = make_task(node, selected, ik_only=ik_only)
        max_solutions = 12 if ik_only else 5
        full_timeout = float(os.environ.get(
            'LEAF_MTC_FULL_TIMEOUT_SECONDS', '20.0'))
        result, timed_out = plan_with_stage_progress(
            task,
            max_solutions,
            '完整任务规划',
            full_timeout,
        )
        if not result and not timed_out:
            print(
                '[leaf_mtc_pinch_demo] First planning attempt returned no '
                'solution; retrying once after ROS discovery settles.',
                flush=True,
            )
            time.sleep(1.0)
            result, timed_out = plan_with_stage_progress(
                task,
                max_solutions,
                '完整任务重试',
                full_timeout,
            )
        if not result:
            print(
                '[leaf_mtc_pinch_demo] No complete solution was found. '
                'Failure details remain in the terminal; RViz was cleared.',
                flush=True,
            )
            task.reset()
            task.enableIntrospection(False)
            return
        else:
            solutions = list(task.solutions)
            solutions.sort(
                key=lambda solution: float(
                    getattr(solution, 'cost', float('inf'))))
            print(
                f'[leaf_mtc_pinch_demo] Generated {len(solutions)} ranked '
                'complete solution(s); publishing all to RViz in ascending '
                'MTC cost order.',
                flush=True,
            )
            for rank, solution in enumerate(solutions, start=1):
                print(
                    '[leaf_mtc_pinch_demo] 发布完整解 '
                    f'{rank}/{len(solutions)}: '
                    f'cost={float(getattr(solution, "cost", 0.0)):.3f}',
                    flush=True,
                )
                task.publish(solution)

            if execute_solution:
                if ik_only:
                    print(
                        '[leaf_mtc_pinch_demo] execute=true 需要完整任务；'
                        'ik_only 模式不会执行。',
                        flush=True,
                    )
                    return
                print(
                    '[leaf_mtc_pinch_demo] 开始在 Gazebo 执行最低代价完整解 '
                    '(预抓取、夹取、松开、回零)。',
                    flush=True,
                )
                execution_result = task.execute(solutions[0])
                execution_code = getattr(execution_result, 'val', None)
                if execution_code != MoveItErrorCodes.SUCCESS:
                    print(
                        '[leaf_mtc_pinch_demo] 完整解执行失败: '
                        f'MoveIt error code={execution_code!r}',
                        flush=True,
                    )
                    return
                print(
                    '[leaf_mtc_pinch_demo] 完整解执行成功：夹爪已松开，'
                    '机械臂已回到垂直 home 位。',
                    flush=True,
                )


        # Keep the introspection services alive while solutions are inspected.
        while not stop_requested[0]:
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        # Ignore repeated signals while plugin-owned objects are torn down.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        if acm_enabled and not ik_prechecker.set_leaf_fingertip_collisions(
            False
        ):
            print(
                '[leaf_mtc_pinch_demo] 警告: 临时叶片指尖碰撞矩阵恢复失败。',
                flush=True,
            )
        # Destroy MTC objects before shutting down plugin loaders.
        del task
        ik_prechecker.close()
        if rclpy.ok():
            rclpy.shutdown()
        del node
        rclcpp.shutdown()


if __name__ == '__main__':
    if len(sys.argv) >= 4 and sys.argv[1] == '--fast-check-child':
        child_input = sys.argv[2]
        child_output = sys.argv[3]
        del sys.argv[1:4]
        sys.exit(fast_check_child(child_input, child_output))
    main()
