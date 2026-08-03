#!/usr/bin/env python3
"""Publish the native Gazebo base wheel joints for robot_state_publisher."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


WHEEL_JOINTS = [
    'right_rear_wheel_joint',
    'right_front_wheel_joint',
    'left_front_wheel_joint',
    'left_rear_wheel_joint',
]


class WheelJointStates(Node):
    """Supply TF-driving states omitted by the native Gazebo base plugin."""

    def __init__(self):
        super().__init__('pro450_myagv_wheel_joint_states')
        self.declare_parameter('publish_rate_hz', 20.0)
        rate = max(1.0, float(
            self.get_parameter('publish_rate_hz').value))
        self._publisher = self.create_publisher(
            JointState, '/joint_states', 10)
        self.create_timer(1.0 / rate, self._publish)
        self.get_logger().info(
            'Publishing four native-base wheel joint states for RViz TF')

    def _publish(self):
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = WHEEL_JOINTS
        # The native Gazebo mobile-base plugin does not expose individual
        # wheel angles. Zero is sufficient to keep each continuous joint in
        # the TF tree; chassis motion still comes from odom -> base_footprint.
        message.position = [0.0] * len(WHEEL_JOINTS)
        self._publisher.publish(message)


def main():
    rclpy.init()
    node = WheelJointStates()
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
