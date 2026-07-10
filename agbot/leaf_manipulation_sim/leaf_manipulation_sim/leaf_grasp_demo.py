#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import Pose
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile

from custom_interfaces.msg import LeafPoseArrays


class LeafGraspDemo(Node):
    """Stage-1 demo node: consume leaf poses and log grasp sequence milestones."""

    POSE_FIELDS = ('poses1', 'poses2', 'poses3', 'poses4', 'poses5')

    def __init__(self) -> None:
        super().__init__('leaf_grasp_demo')
        self.declare_parameter('target_topic', '/target_leaves_multi_pose')
        self.declare_parameter('pose_set_index', 0)
        self.declare_parameter('approach_offset_z', 0.08)
        self.declare_parameter('grasp_finger_position', 0.40)
        self.declare_parameter('open_finger_position', 0.78)

        target_topic = self.get_parameter('target_topic').value
        self.pose_set_index = int(self.get_parameter('pose_set_index').value)
        self.approach_offset_z = float(self.get_parameter('approach_offset_z').value)
        self.grasp_finger_position = float(self.get_parameter('grasp_finger_position').value)
        self.open_finger_position = float(self.get_parameter('open_finger_position').value)

        self.processed = False
        qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.subscription = self.create_subscription(
            LeafPoseArrays, target_topic, self.callback, qos)

        self.get_logger().info(
            'Leaf grasp demo ready. Waiting for LeafPoseArrays on '
            f'{target_topic}')

    def _select_poses(self, msg: LeafPoseArrays):
        index = max(0, min(self.pose_set_index, len(self.POSE_FIELDS) - 1))
        field = self.POSE_FIELDS[index]
        poses = getattr(msg, field)
        return poses if poses else msg.poses1

    def _log_pose(self, label: str, pose: Pose) -> None:
        self.get_logger().info(
            f'{label}: x={pose.position.x:.3f}, '
            f'y={pose.position.y:.3f}, z={pose.position.z:.3f}')

    def callback(self, msg: LeafPoseArrays) -> None:
        if self.processed:
            return

        poses = self._select_poses(msg)
        if not poses:
            self.get_logger().warn('No leaf poses available for demo')
            return

        target = poses[0]
        approach = Pose()
        approach.position.x = target.position.x
        approach.position.y = target.position.y
        approach.position.z = target.position.z + self.approach_offset_z
        approach.orientation = target.orientation

        self.get_logger().info('=== Leaf grasp demo sequence ===')
        self._log_pose('1) Open gripper', target)
        self.get_logger().info(
            f'   finger_joint -> {self.open_finger_position:.2f}')
        self._log_pose('2) Approach pose', approach)
        self._log_pose('3) Grasp pose', target)
        self.get_logger().info(
            f'   finger_joint -> {self.grasp_finger_position:.2f}')
        self.get_logger().info(
            '4) Retreat (future: MoveIt trajectory execution in stage 2)')
        self.processed = True


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LeafGraspDemo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
