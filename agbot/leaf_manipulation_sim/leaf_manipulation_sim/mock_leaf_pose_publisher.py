#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import Pose
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Header

from custom_interfaces.msg import LeafPoseArrays


class MockLeafPosePublisher(Node):
    """Publish synthetic LeafPoseArrays for bench testing without vision."""

    def __init__(self) -> None:
        super().__init__('mock_leaf_pose_publisher')
        self.declare_parameter('output_topic', '/target_leaves_multi_pose')
        self.declare_parameter('frame_id', 'base')
        self.declare_parameter('publish_rate', 1.0)
        self.declare_parameter('leaf_positions', [[0.55, 0.10, 0.70]])

        output_topic = self.get_parameter('output_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        rate = float(self.get_parameter('publish_rate').value)
        self.leaf_positions = self.get_parameter('leaf_positions').value

        qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(LeafPoseArrays, output_topic, qos)
        self.timer = self.create_timer(1.0 / max(rate, 0.1), self.publish_poses)
        self.get_logger().info(
            f'Publishing mock LeafPoseArrays on {output_topic} at {rate} Hz')

    def _make_pose(self, xyz) -> Pose:
        pose = Pose()
        pose.position.x = float(xyz[0])
        pose.position.y = float(xyz[1])
        pose.position.z = float(xyz[2])
        pose.orientation.w = 1.0
        return pose

    def publish_poses(self) -> None:
        msg = LeafPoseArrays()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        poses = [self._make_pose(xyz) for xyz in self.leaf_positions]
        msg.poses1 = poses
        msg.poses2 = poses
        msg.poses3 = poses
        msg.poses4 = poses
        msg.poses5 = poses
        self.publisher.publish(msg)
        self.get_logger().info(f'Published {len(poses)} mock leaf pose(s)')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MockLeafPosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
