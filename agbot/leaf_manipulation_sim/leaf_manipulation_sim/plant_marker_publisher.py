#!/usr/bin/env python3
"""Publish the Gazebo potted plant as a latched RViz mesh marker."""

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker


class PlantMarkerPublisher(Node):
    def __init__(self) -> None:
        super().__init__('plant_marker_publisher')
        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.publisher = self.create_publisher(Marker, '/plant_marker', qos)
        self.timer = self.create_timer(0.5, self.publish_marker)
        self.published = False

    def publish_marker(self) -> None:
        marker = Marker()
        marker.header.frame_id = 'world'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'simple_potted_plant'
        marker.id = 0
        marker.type = Marker.MESH_RESOURCE
        marker.action = Marker.ADD
        marker.pose.position.x = 0.85
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0001
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.10
        marker.scale.y = 0.10
        marker.scale.z = 0.10
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        marker.mesh_resource = (
            'package://leaf_manipulation_sim/models/'
            'simple_potted_plant/meshes/FlowerPot.dae'
        )
        marker.mesh_use_embedded_materials = True
        marker.frame_locked = True
        self.publisher.publish(marker)
        if not self.published:
            self.get_logger().info('Published potted plant marker on /plant_marker')
            self.published = True
        self.timer.cancel()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PlantMarkerPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
