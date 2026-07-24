#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class InitialControllerCommands(Node):
    def __init__(self):
        super().__init__('initial_controller_commands')
        self.arm_pub = self.create_publisher(
            Float64MultiArray, '/arm_position_controller/commands', 10)
        self.gripper_pub = self.create_publisher(
            Float64MultiArray, '/gripper_position_controller/commands', 10)

    def run(self):
        deadline = time.monotonic() + 15.0
        while rclpy.ok() and time.monotonic() < deadline:
            if self.arm_pub.get_subscription_count() and self.gripper_pub.get_subscription_count():
                break
            rclpy.spin_once(self, timeout_sec=0.1)
        else:
            self.get_logger().error('Gazebo controllers did not become ready')
            return False

        arm = Float64MultiArray()
        # SRDF "home": the TM5-900 is vertical, matching the robot model's
        # zero/default state in both Gazebo and RViz.
        arm.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        gripper = Float64MultiArray()
        finger_position = 0.10
        gripper.data = [
            finger_position,
            -finger_position,
            finger_position,
            -finger_position,
            -finger_position,
            finger_position,
        ]
        end_time = time.monotonic() + 2.0
        while rclpy.ok() and time.monotonic() < end_time:
            self.arm_pub.publish(arm)
            self.gripper_pub.publish(gripper)
            rclpy.spin_once(self, timeout_sec=1.0 / 30.0)
        self.get_logger().info('Initial vertical home controller commands sent')
        return True


def main():
    rclpy.init()
    node = InitialControllerCommands()
    ok = node.run()
    node.destroy_node()
    rclpy.shutdown()
    raise SystemExit(0 if ok else 1)


if __name__ == '__main__':
    main()
