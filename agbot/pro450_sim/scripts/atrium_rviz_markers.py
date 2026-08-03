#!/usr/bin/env python3
"""Publish atrium plants + wall boxes for RViz (match Gazebo 回 corridor).

Wall geometry stays in sync with generate_atrium_corridor_sdf.py constants.
"""

from __future__ import annotations

import math
import os

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray
from shape_msgs.msg import SolidPrimitive

from leaf_manipulation_sim.plant_collision_geometry import (
    plant_collision_objects,
)


# Keep in sync with generate_atrium_corridor_sdf.py
OUTER = 5.0
ATRIUM = 3.0
WALL_T = 0.12
WALL_H = 3.0
DOOR_W = 1.2

PLANT_MESH_SCALE = 0.05
PLANT_MESH = (
    'package://leaf_manipulation_sim/models/'
    'simple_potted_plant/meshes/FlowerPot_fortress.obj'
)
PLANT_QX = 0.7071067811865476
PLANT_QW = 0.7071067811865476

PLANT_POSES = [
    (2.5, 4.65),
    (-2.5, 4.65),
    (2.5, -4.65),
    (-2.5, -4.65),
    (4.65, 2.5),
    (4.65, -2.5),
    (-4.65, 2.5),
    (-4.65, -2.5),
]


def _latch_qos() -> QoSProfile:
    return QoSProfile(
        depth=1,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
    )


def _plant_marker(pid: int, x: float, y: float) -> Marker:
    m = Marker()
    m.header.frame_id = 'map'
    m.header.stamp.sec = 0
    m.header.stamp.nanosec = 0
    m.ns = 'simple_potted_plant'
    m.id = pid
    m.type = Marker.MESH_RESOURCE
    m.action = Marker.ADD
    m.pose.position.x = float(x)
    m.pose.position.y = float(y)
    m.pose.position.z = 0.0
    m.pose.orientation.x = PLANT_QX
    m.pose.orientation.w = PLANT_QW
    m.scale.x = m.scale.y = m.scale.z = float(PLANT_MESH_SCALE)
    m.color.r = m.color.g = m.color.b = m.color.a = 1.0
    m.mesh_resource = PLANT_MESH
    m.mesh_use_embedded_materials = True
    m.frame_locked = True
    m.lifetime.sec = 0
    return m


def _plant_label(pid: int, x: float, y: float) -> Marker:
    m = Marker()
    m.header.frame_id = 'map'
    m.ns = 'plant_numbers'
    m.id = pid
    m.type = Marker.TEXT_VIEW_FACING
    m.action = Marker.ADD
    m.pose.position.x = float(x)
    m.pose.position.y = float(y)
    m.pose.position.z = 0.55
    m.pose.orientation.w = 1.0
    m.scale.z = 0.32
    m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.85, 0.05, 1.0
    m.text = str(pid + 1)
    m.frame_locked = True
    return m


def _plant_collision_markers(frame_id: str) -> list[Marker]:
    """Expose the exact pot/leaf MoveIt proxies for all eight plants."""
    markers = []
    previous_pose = os.environ.get('LEAF_PLANT_POSE')
    previous_scale = os.environ.get('LEAF_PLANT_PROXY_SCALE')
    try:
        os.environ['LEAF_PLANT_PROXY_SCALE'] = '0.5'
        marker_id = 0
        for plant_index, (x, y) in enumerate(PLANT_POSES, start=1):
            os.environ['LEAF_PLANT_POSE'] = f'{x} {y} 0 0 0 0'
            for collision in plant_collision_objects():
                for primitive, pose in zip(
                        collision.primitives, collision.primitive_poses):
                    marker = Marker()
                    marker.header.frame_id = frame_id
                    marker.ns = f'plant_{plant_index}_collision'
                    marker.id = marker_id
                    marker_id += 1
                    marker.action = Marker.ADD
                    marker.pose = pose
                    if primitive.type == SolidPrimitive.BOX:
                        marker.type = Marker.CUBE
                        marker.scale.x = primitive.dimensions[0]
                        marker.scale.y = primitive.dimensions[1]
                        marker.scale.z = primitive.dimensions[2]
                    elif primitive.type == SolidPrimitive.CYLINDER:
                        marker.type = Marker.CYLINDER
                        radius = primitive.dimensions[
                            SolidPrimitive.CYLINDER_RADIUS]
                        marker.scale.x = marker.scale.y = 2.0 * radius
                        marker.scale.z = primitive.dimensions[
                            SolidPrimitive.CYLINDER_HEIGHT]
                    else:
                        continue
                    is_pot = collision.id == 'pot_collision'
                    marker.color.r = 0.95 if is_pot else 0.15
                    marker.color.g = 0.35 if is_pot else 0.85
                    marker.color.b = 0.10 if is_pot else 0.25
                    marker.color.a = 0.28 if is_pot else 0.38
                    marker.frame_locked = True
                    markers.append(marker)
    finally:
        if previous_pose is None:
            os.environ.pop('LEAF_PLANT_POSE', None)
        else:
            os.environ['LEAF_PLANT_POSE'] = previous_pose
        if previous_scale is None:
            os.environ.pop('LEAF_PLANT_PROXY_SCALE', None)
        else:
            os.environ['LEAF_PLANT_PROXY_SCALE'] = previous_scale
    return markers


def _cube(ns: str, mid: int, x: float, y: float, z: float,
          sx: float, sy: float, sz: float,
          r: float, g: float, b: float, a: float = 0.55) -> Marker:
    m = Marker()
    m.header.frame_id = 'map'
    m.header.stamp.sec = 0
    m.header.stamp.nanosec = 0
    m.ns = ns
    m.id = mid
    m.type = Marker.CUBE
    m.action = Marker.ADD
    m.pose.position.x = float(x)
    m.pose.position.y = float(y)
    m.pose.position.z = float(z)
    m.pose.orientation.w = 1.0
    m.scale.x = float(sx)
    m.scale.y = float(sy)
    m.scale.z = float(sz)
    m.color.r = float(r)
    m.color.g = float(g)
    m.color.b = float(b)
    m.color.a = float(a)
    m.frame_locked = True
    m.lifetime.sec = 0
    return m


def _wall_markers() -> list[Marker]:
    """Simplified outer + atrium walls (door openings approximated as gaps)."""
    out: list[Marker] = []
    mid = 0
    z = WALL_H / 2.0
    seg = (2 * OUTER - DOOR_W) / 2.0
    # Outer N/S wall segments (door gap at center)
    for y, side in ((OUTER, 'n'), (-OUTER, 's')):
        out.append(_cube('atrium_walls', mid, -OUTER + seg / 2.0, y, z,
                         seg, WALL_T, WALL_H, 0.55, 0.55, 0.58)); mid += 1
        out.append(_cube('atrium_walls', mid, OUTER - seg / 2.0, y, z,
                         seg, WALL_T, WALL_H, 0.55, 0.55, 0.58)); mid += 1
        # closed door leaf
        out.append(_cube('atrium_doors', mid, 0.0, y, 1.1,
                         DOOR_W - 0.04, 0.045, 2.2, 0.35, 0.22, 0.12, 0.8)); mid += 1
    # Outer E/W wall segments
    ew_seg = (2 * OUTER - WALL_T - DOOR_W) / 2.0
    for x, side in ((OUTER, 'e'), (-OUTER, 'w')):
        out.append(_cube('atrium_walls', mid, x, OUTER - WALL_T / 2.0 - ew_seg / 2.0, z,
                         WALL_T, ew_seg, WALL_H, 0.55, 0.55, 0.58)); mid += 1
        out.append(_cube('atrium_walls', mid, x, -OUTER + WALL_T / 2.0 + ew_seg / 2.0, z,
                         WALL_T, ew_seg, WALL_H, 0.55, 0.55, 0.58)); mid += 1
        out.append(_cube('atrium_doors', mid, x, 0.0, 1.1,
                         0.045, DOOR_W - 0.04, 2.2, 0.35, 0.22, 0.12, 0.8)); mid += 1
    # Atrium solid walls
    Al = 2 * ATRIUM
    out.append(_cube('atrium_inner', mid, 0.0, ATRIUM, z,
                     Al + WALL_T, WALL_T, WALL_H, 0.45, 0.48, 0.50)); mid += 1
    out.append(_cube('atrium_inner', mid, 0.0, -ATRIUM, z,
                     Al + WALL_T, WALL_T, WALL_H, 0.45, 0.48, 0.50)); mid += 1
    out.append(_cube('atrium_inner', mid, ATRIUM, 0.0, z,
                     WALL_T, Al - WALL_T, WALL_H, 0.45, 0.48, 0.50)); mid += 1
    out.append(_cube('atrium_inner', mid, -ATRIUM, 0.0, z,
                     WALL_T, Al - WALL_T, WALL_H, 0.45, 0.48, 0.50)); mid += 1
    # Floor ring hint (thin, low alpha)
    ring_w = OUTER - ATRIUM
    out.append(_cube('atrium_floor', mid, 0.0, (OUTER + ATRIUM) / 2.0, 0.01,
                     2 * OUTER, ring_w, 0.02, 0.6, 0.6, 0.58, 0.25)); mid += 1
    out.append(_cube('atrium_floor', mid, 0.0, -(OUTER + ATRIUM) / 2.0, 0.01,
                     2 * OUTER, ring_w, 0.02, 0.6, 0.6, 0.58, 0.25)); mid += 1
    out.append(_cube('atrium_floor', mid, (OUTER + ATRIUM) / 2.0, 0.0, 0.01,
                     ring_w, 2 * ATRIUM, 0.02, 0.6, 0.6, 0.58, 0.25)); mid += 1
    out.append(_cube('atrium_floor', mid, -(OUTER + ATRIUM) / 2.0, 0.0, 0.01,
                     ring_w, 2 * ATRIUM, 0.02, 0.6, 0.6, 0.58, 0.25)); mid += 1
    _ = math  # silence unused if removed later
    return out


class AtriumRvizMarkers(Node):
    def __init__(self):
        super().__init__('atrium_rviz_markers')
        self.declare_parameter('publish_walls', True)
        self.declare_parameter('publish_collision_boxes', False)
        self.declare_parameter('publish_plant_labels', False)
        self.declare_parameter('frame_id', 'world')
        publish_walls = bool(self.get_parameter('publish_walls').value)
        publish_collision_boxes = bool(
            self.get_parameter('publish_collision_boxes').value)
        publish_plant_labels = bool(
            self.get_parameter('publish_plant_labels').value)
        frame_id = str(self.get_parameter('frame_id').value)

        self.pub_plants = self.create_publisher(
            MarkerArray, '/atrium_plant_markers', _latch_qos())
        self.pub_walls = self.create_publisher(
            MarkerArray, '/atrium_wall_markers', _latch_qos())
        self.pub_labels = self.create_publisher(
            MarkerArray, '/atrium_plant_labels', _latch_qos())
        self.pub_collisions = self.create_publisher(
            MarkerArray, '/plant_collision_markers', _latch_qos())

        self._plants = MarkerArray()
        self._plants.markers = [
            _plant_marker(i, x, y) for i, (x, y) in enumerate(PLANT_POSES)
        ]
        for m in self._plants.markers:
            m.header.frame_id = frame_id

        self._labels = MarkerArray()
        if publish_plant_labels:
            self._labels.markers = [
                _plant_label(i, x, y)
                for i, (x, y) in enumerate(PLANT_POSES)]
            for m in self._labels.markers:
                m.header.frame_id = frame_id

        self._collisions = MarkerArray()
        if publish_collision_boxes:
            self._collisions.markers = _plant_collision_markers(frame_id)

        self._walls = MarkerArray()
        if publish_walls:
            self._walls.markers = _wall_markers()
            for m in self._walls.markers:
                m.header.frame_id = frame_id

        self._publish()
        self.create_timer(1.0, self._publish)
        self.get_logger().info(
            f'Publishing {len(self._plants.markers)} plants + '
            f'{len(self._labels.markers)} labels + '
            f'{len(self._collisions.markers)} collision proxies + '
            f'{len(self._walls.markers)} wall/floor markers '
            f'(frame={frame_id})')

    def _publish(self):
        self.pub_plants.publish(self._plants)
        if self._labels.markers:
            self.pub_labels.publish(self._labels)
        if self._collisions.markers:
            self.pub_collisions.publish(self._collisions)
        if self._walls.markers:
            self.pub_walls.publish(self._walls)


def main():
    rclpy.init()
    node = AtriumRvizMarkers()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
