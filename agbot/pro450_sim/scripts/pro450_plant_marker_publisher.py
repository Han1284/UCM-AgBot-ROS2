#!/usr/bin/env python3
"""Publish the Pro450-scaled plant visual for RViz."""

import os

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from shape_msgs.msg import SolidPrimitive
from visualization_msgs.msg import Marker, MarkerArray

from leaf_manipulation_sim.plant_collision_geometry import (
    plant_collision_objects,
)


class Pro450PlantMarkerPublisher(Node):
    def __init__(self):
        super().__init__('pro450_plant_marker_publisher')
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.publisher = self.create_publisher(Marker, '/plant_marker', qos)
        self.collision_publisher = self.create_publisher(
            MarkerArray, '/plant_collision_markers', qos)
        self.timer = self.create_timer(0.5, self.publish_marker)

    def publish_marker(self):
        marker = Marker()
        marker.header.frame_id = 'world'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'simple_potted_plant'
        marker.id = 0
        marker.type = Marker.MESH_RESOURCE
        marker.action = Marker.ADD
        marker.pose.position.x = 0.425
        # Gazebo uses the Fortress OBJ conversion, whose Y-up export is
        # restored with this +90 degree X rotation in model.sdf.  Use the
        # identical mesh and rotation in RViz so the two views cannot diverge.
        marker.pose.orientation.x = 0.7071067811865476
        marker.pose.orientation.w = 0.7071067811865476
        # Match the independent half-scale Pro450 Gazebo plant exactly.
        marker.scale.x = marker.scale.y = marker.scale.z = 0.05
        marker.color.r = marker.color.g = marker.color.b = marker.color.a = 1.0
        marker.mesh_resource = ('package://leaf_manipulation_sim/models/'
                                'simple_potted_plant/meshes/'
                                'FlowerPot_fortress.obj')
        marker.mesh_use_embedded_materials = True
        marker.frame_locked = True
        self.publisher.publish(marker)
        self.publish_collision_markers()
        self.timer.cancel()

    def publish_collision_markers(self):
        # Use exactly the same world, frame and half-scale as the Pro450 leaf
        # pipeline.  These environment variables are local to this publisher.
        os.environ['LEAF_PLANT_WORLD_PACKAGE'] = 'pro450_sim'
        os.environ['LEAF_PLANT_WORLD_RELATIVE_PATH'] = (
            'worlds/pro450_leaf_bench.world')
        os.environ['LEAF_PLANT_FRAME'] = 'base_root'
        os.environ['LEAF_PLANT_PROXY_SCALE'] = '0.5'

        message = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        for marker_id, collision_object in enumerate(
                plant_collision_objects()):
            primitive = collision_object.primitives[0]
            collision_marker = Marker()
            collision_marker.header.frame_id = collision_object.header.frame_id
            collision_marker.header.stamp = stamp
            collision_marker.ns = 'plant_collision_proxies'
            collision_marker.id = marker_id
            collision_marker.action = Marker.ADD
            collision_marker.pose = collision_object.primitive_poses[0]
            collision_marker.frame_locked = True

            if primitive.type == SolidPrimitive.BOX:
                collision_marker.type = Marker.CUBE
                collision_marker.scale.x = primitive.dimensions[
                    SolidPrimitive.BOX_X]
                collision_marker.scale.y = primitive.dimensions[
                    SolidPrimitive.BOX_Y]
                collision_marker.scale.z = primitive.dimensions[
                    SolidPrimitive.BOX_Z]
                collision_marker.color.r = 0.1
                collision_marker.color.g = 0.8
                collision_marker.color.b = 1.0
                collision_marker.color.a = 0.24
            elif primitive.type == SolidPrimitive.CYLINDER:
                collision_marker.type = Marker.CYLINDER
                radius = primitive.dimensions[
                    SolidPrimitive.CYLINDER_RADIUS]
                collision_marker.scale.x = 2.0 * radius
                collision_marker.scale.y = 2.0 * radius
                collision_marker.scale.z = primitive.dimensions[
                    SolidPrimitive.CYLINDER_HEIGHT]
                collision_marker.color.r = 1.0
                collision_marker.color.g = 0.35
                collision_marker.color.b = 0.1
                collision_marker.color.a = 0.16
            else:
                continue
            message.markers.append(collision_marker)
        self.collision_publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = Pro450PlantMarkerPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
