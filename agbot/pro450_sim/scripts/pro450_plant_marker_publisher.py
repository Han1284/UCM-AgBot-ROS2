#!/usr/bin/env python3
"""Publish the Pro450-scaled plant visual for RViz."""

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker


class Pro450PlantMarkerPublisher(Node):
    def __init__(self):
        super().__init__('pro450_plant_marker_publisher')
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.publisher = self.create_publisher(Marker, '/plant_marker', qos)
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
        self.timer.cancel()


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
