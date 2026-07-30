#!/usr/bin/env python3
"""Publish the Pro 450 arm and F100 master-joint state for static simulation."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class StaticJointStates(Node):
    def __init__(self):
        super().__init__('pro450_static_joint_states')
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', True)
        self._publisher = self.create_publisher(JointState, 'joint_states', 10)
        self._timer = self.create_timer(0.1, self._publish)

    def _publish(self):
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'gripper_controller']
        message.position = [0.0] * len(message.name)
        self._publisher.publish(message)


def main():
    rclpy.init()
    node = StaticJointStates()
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
