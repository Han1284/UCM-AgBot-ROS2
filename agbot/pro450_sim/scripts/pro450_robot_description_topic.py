#!/usr/bin/env python3
"""Publish expanded robot_description for Humble RViz and joint_state_publisher.

ROS 2 Humble's joint_state_publisher waits for a transient-local String topic
when no URDF file is given on its command line.  Passing robot_description only
as a node parameter therefore leaves it unconfigured.
"""

import rclpy
import xml.etree.ElementTree as ET
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


class RobotDescriptionTopic(Node):
    def __init__(self):
        super().__init__('pro450_robot_description_topic')
        self.declare_parameter('robot_description', '')
        self._description = str(self.get_parameter('robot_description').value)
        try:
            root = ET.fromstring(self._description)
        except ET.ParseError as exc:
            raise RuntimeError(
                f'robot_description is empty or invalid XML: {exc}') from exc
        if root.tag != 'robot':
            raise RuntimeError(
                f'robot_description root is {root.tag!r}, expected "robot"')

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._publisher = self.create_publisher(String, 'robot_description', qos)
        self._message = String(data=self._description)
        self._publish()
        # RViz Humble commonly requests volatile durability.  A low-rate repeat
        # lets a late volatile subscriber receive the description as well.
        self.create_timer(2.0, self._publish)

    def _publish(self):
        self._publisher.publish(self._message)


def main():
    rclpy.init()
    node = RobotDescriptionTopic()
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
