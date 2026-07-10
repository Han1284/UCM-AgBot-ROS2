#!/usr/bin/env python3
"""Publish sanitized robot_description on /robot_description (TRANSIENT_LOCAL)."""

import subprocess
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String


class RobotDescriptionPublisher(Node):
    def __init__(self) -> None:
        super().__init__('robot_description_publisher')
        self.declare_parameter('urdf_xacro', '')
        self.declare_parameter('sanitize_script', '')

        xacro_path = self.get_parameter('urdf_xacro').get_parameter_value().string_value
        sanitize_script = self.get_parameter('sanitize_script').get_parameter_value().string_value
        if not xacro_path or not sanitize_script:
            self.get_logger().error('urdf_xacro and sanitize_script parameters are required')
            sys.exit(1)

        cmd = f'xacro "{xacro_path}" | python3 "{sanitize_script}"'
        result = subprocess.run(
            cmd, shell=True, capture_output=True, text=True, check=False)
        if result.returncode != 0:
            self.get_logger().error(f'URDF generation failed: {result.stderr}')
            sys.exit(1)

        description = result.stdout
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(String, 'robot_description', qos)
        self.timer = self.create_timer(0.5, self._publish_once)
        self._description = description
        self._published = False
        self.get_logger().info(f'Generated URDF ({len(description)} bytes)')

    def _publish_once(self) -> None:
        if self._published:
            return
        msg = String()
        msg.data = self._description
        self.publisher.publish(msg)
        self._published = True
        self.get_logger().info('Published /robot_description')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RobotDescriptionPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
