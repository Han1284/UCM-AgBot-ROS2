#!/usr/bin/env python3

"""Validate projected landing-point metadata and RViz markers headlessly."""

import math
import sys
import time

from custom_interfaces.msg import LeafPoseArrays
from leaf_extraction.leaf_surface_projection import (
    box_broad_face_residual,
    leaf_collision_groups,
)
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from visualization_msgs.msg import Marker, MarkerArray


def _distance(first, second):
    return math.sqrt(sum(
        (first[index] - second[index]) ** 2 for index in range(3)))


def _point_tuple(point):
    return point.x, point.y, point.z


def validate(message, marker_array):
    """Return validation errors for one synchronized retained publication."""
    errors = []
    count = len(message.poses1)
    metadata = {
        'leaf_ids': message.leaf_ids,
        'longitudinal_ratios': message.longitudinal_ratios,
        'projection_distances': message.projection_distances,
        'collision_leaf_ids': message.collision_leaf_ids,
        'local_leaf_widths': message.local_leaf_widths,
    }
    if count < 6:
        errors.append(f'only {count} projected candidates')
    for name, values in metadata.items():
        if len(values) != count:
            errors.append(
                f'{name} length {len(values)} != candidate count {count}')
    if len(set(message.leaf_ids)) < 2:
        errors.append('projected candidates cover fewer than two leaves')

    spheres = {
        marker.id: marker
        for marker in marker_array.markers
        if marker.action == Marker.ADD
        and marker.ns == 'ranked_grasp_points'
    }
    labels = {
        marker.id: marker
        for marker in marker_array.markers
        if marker.action == Marker.ADD
        and marker.ns == 'projected_grasp_labels'
    }
    expected_ids = set(range(count))
    if set(spheres) != expected_ids:
        errors.append('sphere marker IDs are not continuous from zero')
    if set(labels) != expected_ids:
        errors.append('numeric label IDs are not continuous from zero')

    collision_groups = leaf_collision_groups()
    for index in range(min(
        count,
        len(message.longitudinal_ratios),
        len(message.projection_distances),
        len(message.collision_leaf_ids),
        len(message.local_leaf_widths),
    )):
        pose_point = _point_tuple(message.poses1[index].position)
        ratio = float(message.longitudinal_ratios[index])
        projection_distance = float(message.projection_distances[index])
        width = float(message.local_leaf_widths[index])
        collision_leaf_id = message.collision_leaf_ids[index]
        if ratio < 0.30 - 1e-6:
            errors.append(
                f'point {index + 1} longitudinal ratio {ratio:.3f} < 0.30')
        if projection_distance > 1.25 * width + 1e-6:
            errors.append(
                f'point {index + 1} projection exceeds dynamic width limit')
        objects = collision_groups.get(collision_leaf_id, [])
        surface_gap = min(
            (
                box_broad_face_residual(pose_point, collision_object)
                for collision_object in objects
            ),
            default=float('inf'),
        )
        if surface_gap > 1e-5:
            errors.append(
                f'point {index + 1} is {surface_gap:.6f} m off the broad '
                f'face of {collision_leaf_id}')
        sphere = spheres.get(index)
        if sphere is not None and _distance(
            pose_point, _point_tuple(sphere.pose.position)
        ) > 1e-7:
            errors.append(
                f'point {index + 1} marker differs from MTC contact point')
        label = labels.get(index)
        if label is not None and label.text != str(index + 1):
            errors.append(f'point {index + 1} has incorrect numeric label')
    return errors


def main():
    rclpy.init()
    node = rclpy.create_node('projected_candidate_validator')
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    messages = []
    marker_arrays = []
    node.create_subscription(
        LeafPoseArrays,
        '/target_leaves_multi_pose',
        messages.append,
        qos,
    )
    node.create_subscription(
        MarkerArray,
        '/leaf_perception/projected_grasp_candidates',
        marker_arrays.append,
        qos,
    )
    deadline = time.monotonic() + 10.0
    while (
        rclpy.ok()
        and time.monotonic() < deadline
        and (not messages or not marker_arrays)
    ):
        rclpy.spin_once(node, timeout_sec=0.2)
    errors = (
        ['timed out waiting for retained candidate topics']
        if not messages or not marker_arrays
        else validate(messages[-1], marker_arrays[-1])
    )
    if errors:
        for error in errors:
            print(f'[projected_candidate_validator] FAIL: {error}')
    else:
        message = messages[-1]
        print(
            '[projected_candidate_validator] PASS: '
            f'{len(message.poses1)} projected candidates across '
            f'{len(set(message.leaf_ids))} perceived leaves; '
            'surface, metadata, marker coordinates and labels agree.')
    node.destroy_node()
    rclpy.shutdown()
    return 1 if errors else 0


if __name__ == '__main__':
    sys.exit(main())
