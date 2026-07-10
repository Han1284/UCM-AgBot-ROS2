#!/usr/bin/env python3
"""Publish a constant Panda joint state so RViz shows the robot in a default pose."""

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState

JOINT_NAMES = [
    'panda_joint1',
    'panda_joint2',
    'panda_joint3',
    'panda_joint4',
    'panda_joint5',
    'panda_joint6',
    'panda_joint7',
    'panda_finger_joint1',
    'panda_finger_joint2',
]

DEFAULT_POSITIONS = [
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.04,
    0.04,
]


class PandaStaticJointStates(Node):
    def __init__(self) -> None:
        super().__init__('panda_static_joint_states')
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', True)
        if not self.has_parameter('publish_rate'):
            self.declare_parameter('publish_rate', 20.0)

        self._use_sim_time = bool(self.get_parameter('use_sim_time').value)
        rate = float(self.get_parameter('publish_rate').value)

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

        if self.get_clock().now().nanoseconds > 0:
            self._sim_clock_ready = True
            self._clock_check.cancel()
            self.get_logger().info('Simulation clock ready, publishing Panda joint_states')

    def _publish(self) -> None:
        if not self._sim_clock_ready:
            return

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES
        msg.position = DEFAULT_POSITIONS
        self.publisher.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PandaStaticJointStates()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
