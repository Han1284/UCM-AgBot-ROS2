#!/usr/bin/env python3

from typing import List

import rclpy
from geometry_msgs.msg import Pose, PoseArray
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile

from custom_interfaces.msg import LeafPoseArrays


class LeafPoseAdapter(Node):
    """Adapt LeafPoseArrays from vision into a PoseArray for downstream consumers."""

    POSE_FIELDS = ('poses1', 'poses2', 'poses3', 'poses4', 'poses5')

    def __init__(self) -> None:
        super().__init__('leaf_pose_adapter')
        self.declare_parameter('input_topic', '/target_leaves_multi_pose')
        self.declare_parameter('output_topic', '/leaf_manipulation/target_poses')
        self.declare_parameter('output_frame', 'base')
        self.declare_parameter('pose_set_index', 0)

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.output_frame = self.get_parameter('output_frame').value
        self.pose_set_index = int(self.get_parameter('pose_set_index').value)

        qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(PoseArray, output_topic, qos)
        self.subscription = self.create_subscription(
            LeafPoseArrays, input_topic, self.callback, qos)

        self.get_logger().info(
            f'Listening on {input_topic}, publishing PoseArray on {output_topic}')

    def _select_poses(self, msg: LeafPoseArrays) -> List[Pose]:
        index = max(0, min(self.pose_set_index, len(self.POSE_FIELDS) - 1))
        field = self.POSE_FIELDS[index]
        poses = getattr(msg, field)
        return poses if poses else msg.poses1

    def callback(self, msg: LeafPoseArrays) -> None:
        poses = self._select_poses(msg)
        if not poses:
            self.get_logger().warn('Received LeafPoseArrays with no poses')
            return

        output = PoseArray()
        output.header = msg.header
        output.header.frame_id = self.output_frame
        output.poses = poses
        self.publisher.publish(output)
        self.get_logger().info(f'Published {len(poses)} adapted leaf pose(s)')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LeafPoseAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
