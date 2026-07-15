#!/usr/bin/env python3
"""Publish configurable joint states, optionally for a finite duration."""

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
DEFAULT_POSITIONS = [0.0] * len(ARM_JOINTS)


class StaticJointStates(Node):
    def __init__(self) -> None:
        super().__init__('static_joint_states')
        # Foxy may inject use_sim_time from launch before node init finishes.
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', True)
        if not self.has_parameter('publish_rate'):
            self.declare_parameter('publish_rate', 30.0)
        if not self.has_parameter('duration_sec'):
            self.declare_parameter('duration_sec', 0.0)
        if not self.has_parameter('joint_names'):
            self.declare_parameter('joint_names', ARM_JOINTS)
        if not self.has_parameter('joint_positions'):
            self.declare_parameter('joint_positions', DEFAULT_POSITIONS)

        rate = float(self.get_parameter('publish_rate').value)
        self._use_sim_time = self.get_parameter('use_sim_time').value
        self._duration_sec = float(self.get_parameter('duration_sec').value)
        self._joint_names = list(self.get_parameter('joint_names').value)
        self._joint_positions = [
            float(value) for value in self.get_parameter('joint_positions').value
        ]
        if len(self._joint_names) != len(self._joint_positions):
            raise ValueError('joint_names and joint_positions must have the same length')
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.publisher = self.create_publisher(JointState, 'joint_states', qos)
        self.timer = self.create_timer(1.0 / max(rate, 1.0), self._publish)
        self._sim_clock_ready = False
        self._clock_check = self.create_timer(0.2, self._check_clock)
        self._publish_started = False
        self._start_time_ns = 0

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
        if not self._publish_started:
            self._publish_started = True
            self._start_time_ns = self.get_clock().now().nanoseconds

        if self._duration_sec > 0.0:
            elapsed_sec = (
                self.get_clock().now().nanoseconds - self._start_time_ns
            ) / 1e9
            if elapsed_sec > self._duration_sec:
                self.get_logger().info('Initial joint state publishing complete, shutting down')
                rclpy.shutdown()
                return

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self._joint_names
        msg.position = self._joint_positions
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
