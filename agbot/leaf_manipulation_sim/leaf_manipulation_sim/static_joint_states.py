#!/usr/bin/env python3
"""Publish constant zero joint_states, synchronized to simulation time."""

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState

ARM_JOINTS = [
    'joint_1',
    'joint_2',
    'joint_3',
    'joint_4',
    'joint_5',
    'joint_6',
]


class StaticJointStates(Node):
    def __init__(self) -> None:
        super().__init__('static_joint_states')
        # Foxy may inject use_sim_time from launch before node init finishes.
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', True)
        if not self.has_parameter('publish_rate'):
            self.declare_parameter('publish_rate', 30.0)

        rate = float(self.get_parameter('publish_rate').value)
        self._use_sim_time = self.get_parameter('use_sim_time').value
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.publisher = self.create_publisher(JointState, 'joint_states', qos)
        self.timer = self.create_timer(1.0 / max(rate, 1.0), self._publish)
        self._sim_clock_ready = False
        self._clock_check = self.create_timer(0.2, self._check_clock)

    def _check_clock(self) -> None:
        if not self._use_sim_time:
            self._sim_clock_ready = True
            self._clock_check.cancel()
            return
        now = self.get_clock().now()
        if now.nanoseconds > 0:
            self._sim_clock_ready = True
            self._clock_check.cancel()
            self.get_logger().info('Simulation clock ready, publishing joint_states')

    def _publish(self) -> None:
        if not self._sim_clock_ready:
            return
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ARM_JOINTS
        msg.position = [0.0] * len(ARM_JOINTS)
        self.publisher.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = StaticJointStates()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
