#!/usr/bin/env python3
"""Drive myAGV Pro forward 5 m, turn left then go 3 m, then move joint2 by 90 deg."""

import math
import sys
import time

import rclpy
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def wrap_pi(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


class MotionDemo(Node):
    def __init__(self):
        super().__init__('pro450_myagv_motion_demo')
        self.declare_parameter('forward_m', 5.0)
        self.declare_parameter('after_turn_m', 3.0)
        self.declare_parameter('turn_rad', math.pi / 2.0)  # left 90 deg
        self.declare_parameter('linear_speed', 0.25)
        self.declare_parameter('angular_speed', 0.35)
        self.declare_parameter('joint2_delta_rad', math.pi / 2.0)
        self.declare_parameter('arm_duration_sec', 4.0)

        self._cmd = self.create_publisher(Twist, 'cmd_vel', 10)
        self._odom = None
        self._joints = {}
        self.create_subscription(Odometry, 'odom', self._on_odom, 10)
        self.create_subscription(JointState, 'joint_states', self._on_joints, 10)
        self._arm = ActionClient(
            self, FollowJointTrajectory, '/arm_trajectory_controller/follow_joint_trajectory')

    def _on_odom(self, msg: Odometry):
        self._odom = msg

    def _on_joints(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            self._joints[name] = pos

    def _wait(self, predicate, timeout_s, label):
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if predicate():
                return True
        self.get_logger().error(f'Timeout waiting for {label}')
        return False

    def _yaw(self):
        q = self._odom.pose.pose.orientation
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def _xy(self):
        p = self._odom.pose.pose.position
        return p.x, p.y

    def _stop(self):
        self._cmd.publish(Twist())

    def _drive_distance(self, distance_m: float, speed: float) -> bool:
        if not self._wait(lambda: self._odom is not None, 10.0, 'odom'):
            return False
        x0, y0 = self._xy()
        twist = Twist()
        twist.linear.x = abs(speed) if distance_m >= 0.0 else -abs(speed)
        self.get_logger().info(f'Driving {distance_m:.2f} m at {twist.linear.x:.2f} m/s')
        deadline = time.monotonic() + abs(distance_m) / max(abs(speed), 0.05) + 15.0
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
            self._cmd.publish(twist)
            x, y = self._xy()
            if math.hypot(x - x0, y - y0) >= abs(distance_m) - 0.02:
                break
        self._stop()
        time.sleep(0.3)
        x, y = self._xy()
        self.get_logger().info(f'Stopped after {math.hypot(x - x0, y - y0):.3f} m')
        return True

    def _turn_left(self, delta_rad: float, speed: float) -> bool:
        if not self._wait(lambda: self._odom is not None, 10.0, 'odom'):
            return False
        yaw0 = self._yaw()
        target = wrap_pi(yaw0 + delta_rad)
        twist = Twist()
        twist.angular.z = abs(speed) if delta_rad >= 0.0 else -abs(speed)
        self.get_logger().info(f'Turning {math.degrees(delta_rad):.1f} deg')
        deadline = time.monotonic() + abs(delta_rad) / max(abs(speed), 0.05) + 15.0
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
            self._cmd.publish(twist)
            if abs(wrap_pi(self._yaw() - target)) < 0.03:
                break
        self._stop()
        time.sleep(0.3)
        self.get_logger().info(f'Yaw now {math.degrees(self._yaw()):.1f} deg')
        return True

    def _move_joint2(self, delta_rad: float, duration_sec: float) -> bool:
        if not self._wait(lambda: 'joint2' in self._joints, 15.0, 'joint2 state'):
            return False
        if not self._arm.wait_for_server(timeout_sec=15.0):
            self.get_logger().error('arm_trajectory_controller action server not available')
            return False

        start = [self._joints.get(f'joint{i}', 0.0) for i in range(1, 7)]
        goal_positions = list(start)
        goal_positions[1] = start[1] + delta_rad
        # joint2 hard limit ±2.1816
        goal_positions[1] = max(-2.1816, min(2.1816, goal_positions[1]))

        traj = JointTrajectory()
        traj.joint_names = [f'joint{i}' for i in range(1, 7)]
        point = JointTrajectoryPoint()
        point.positions = goal_positions
        point.time_from_start.sec = int(duration_sec)
        point.time_from_start.nanosec = int((duration_sec % 1.0) * 1e9)
        traj.points = [point]

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        self.get_logger().info(
            f'Moving joint2 {math.degrees(start[1]):.1f} → {math.degrees(goal_positions[1]):.1f} deg')
        future = self._arm.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error('joint2 trajectory rejected')
            return False
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=duration_sec + 10.0)
        result = result_future.result()
        if result is None:
            self.get_logger().error('joint2 trajectory result missing')
            return False
        self.get_logger().info(f'joint2 done, error_code={result.result.error_code}')
        return result.result.error_code == FollowJointTrajectory.Result.SUCCESSFUL

    def run(self) -> int:
        forward_m = float(self.get_parameter('forward_m').value)
        after_turn_m = float(self.get_parameter('after_turn_m').value)
        turn_rad = float(self.get_parameter('turn_rad').value)
        v = float(self.get_parameter('linear_speed').value)
        w = float(self.get_parameter('angular_speed').value)
        j2 = float(self.get_parameter('joint2_delta_rad').value)
        arm_t = float(self.get_parameter('arm_duration_sec').value)

        if not self._drive_distance(forward_m, v):
            return 1
        if not self._turn_left(turn_rad, w):
            return 1
        if not self._drive_distance(after_turn_m, v):
            return 1
        if not self._move_joint2(j2, arm_t):
            return 1
        self.get_logger().info('Motion sequence complete')
        return 0


def main():
    rclpy.init()
    node = MotionDemo()
    try:
        code = node.run()
    finally:
        try:
            node.destroy_node()
        except Exception:  # noqa: BLE001
            pass
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
