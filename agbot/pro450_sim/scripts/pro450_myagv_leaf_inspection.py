#!/usr/bin/env python3
"""Navigate to one numbered plant, execute leaf pinch, then return home."""

from __future__ import annotations

import json
import math
import os
from pathlib import Path
import subprocess
import time

from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from moveit_task_constructor_msgs.msg import TaskDescription
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import yaml


ARM_JOINTS = [f'joint{i}' for i in range(1, 7)]


def _yaw_to_quat(yaw: float):
    return 0.0, 0.0, math.sin(0.5 * yaw), math.cos(0.5 * yaw)


def _quat_to_yaw(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


class SinglePlantLeafInspection(Node):
    def __init__(self):
        super().__init__('pro450_myagv_leaf_inspection')
        self.declare_parameter('plants_file', '')
        self.declare_parameter('plant_id', 0)
        self.declare_parameter('startup_delay_sec', 3.0)
        self.declare_parameter('settle_linear_speed', 0.01)
        self.declare_parameter('settle_angular_speed', 0.02)
        self.declare_parameter('settle_duration_sec', 1.0)
        self.declare_parameter('pipeline_timeout_sec', 420.0)
        self.declare_parameter('execute', True)
        self.declare_parameter('home_tolerance', 0.05)
        # 0.56 m keeps the wrist RGB-D canopy in view while remaining 135 mm
        # farther than the 0.425 m fixed-bench baseline.
        self.declare_parameter('precision_dock_distance', 0.56)
        self.declare_parameter('precision_dock_speed', 0.025)
        self.declare_parameter('precision_dock_timeout_sec', 10.0)

        config_path = Path(str(self.get_parameter('plants_file').value))
        if not config_path.is_file():
            raise RuntimeError(f'plants_file missing: {config_path}')
        with config_path.open(encoding='utf-8') as stream:
            config = yaml.safe_load(stream)
        plant_id = int(self.get_parameter('plant_id').value)
        plants = {int(item['id']): item for item in config.get('plants', [])}
        if plant_id not in plants:
            raise RuntimeError(f'plant_id must be one of {sorted(plants)}')
        self._plant = plants[plant_id]
        self._initial = config['initial_pose']
        self._frame = str(config.get('frame_id', 'map'))
        self._execute = bool(self.get_parameter('execute').value)
        if not self._execute:
            raise RuntimeError(
                'The inspection mission requires execute:=true; use the '
                'standalone leaf pipeline for planning-only debugging.')

        self._startup_delay = float(
            self.get_parameter('startup_delay_sec').value)
        self._linear_limit = float(
            self.get_parameter('settle_linear_speed').value)
        self._angular_limit = float(
            self.get_parameter('settle_angular_speed').value)
        self._settle_duration = float(
            self.get_parameter('settle_duration_sec').value)
        self._pipeline_timeout = float(
            self.get_parameter('pipeline_timeout_sec').value)
        self._home_tolerance = float(
            self.get_parameter('home_tolerance').value)
        self._precision_distance = float(
            self.get_parameter('precision_dock_distance').value)
        self._precision_speed = float(
            self.get_parameter('precision_dock_speed').value)
        self._precision_timeout = float(
            self.get_parameter('precision_dock_timeout_sec').value)

        self._nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._arm = ActionClient(
            self, FollowJointTrajectory,
            '/arm_trajectory_controller/follow_joint_trajectory')
        # Nav2 bringup feeds cmd_vel_nav through velocity_smoother and exposes
        # the smoothed command on /cmd_vel, which is bridged to Gazebo.
        self._cmd_vel = self.create_publisher(Twist, '/cmd_vel_nav', 10)
        self._mtc_task_ids = set()
        self._mtc_clear_started = None
        mtc_qos = QoSProfile(
            depth=20,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._mtc_description_pub = self.create_publisher(
            TaskDescription, '/task_description', mtc_qos)
        self.create_subscription(
            TaskDescription,
            '/task_description',
            self._on_mtc_description,
            mtc_qos,
        )
        self.create_subscription(Odometry, '/odom', self._on_odom, 20)
        self.create_subscription(
            JointState, '/joint_states', self._on_joint_state, 20)
        self._latest_odom = None
        self._arm_positions = {}
        self._state = 'startup'
        self._started_at = time.monotonic()
        self._settled_since = None
        self._pipeline = None
        self._pipeline_started = None
        self._result_path = Path(
            f'/tmp/pro450_leaf_inspection_{os.getpid()}.json')
        self._home_goal_sent = False
        self._home_started = None
        self._precision_started = None
        self.create_timer(0.1, self._tick)
        self.get_logger().info(
            'Single-plant inspection selected: '
            f'#{plant_id} {self._plant["name"]}; '
            'mission=approach -> execute leaf pipeline -> arm home -> '
            'return initial base pose')

    def _on_odom(self, message):
        self._latest_odom = message

    def _on_mtc_description(self, message):
        """Remember displayed MTC tasks so they can be removed before driving."""
        if not message.task_id:
            return
        if message.stages:
            self._mtc_task_ids.add(message.task_id)
        # Keep IDs after an intermediate reset as well.  Publishing the empty
        # description again before driving is harmless and protects against a
        # reset that was emitted while RViz was reconnecting.

    def _clear_mtc_visualization(self):
        """Remove stopped-base scenes before base_footprint starts moving."""
        task_ids = tuple(self._mtc_task_ids)
        for task_id in task_ids:
            reset = TaskDescription()
            reset.task_id = task_id
            self._mtc_description_pub.publish(reset)
        self._mtc_clear_started = time.monotonic()
        self._state = 'clear_mtc'
        self.get_logger().info(
            f'Clearing {len(task_ids)} dock-relative MTC RViz task(s) '
            'before base return; map-fixed plant proxies remain visible.')

    def _on_joint_state(self, message):
        self._arm_positions.update(zip(message.name, message.position))

    def _pose(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.frame_id = self._frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        _, _, pose.pose.orientation.z, pose.pose.orientation.w = (
            _yaw_to_quat(float(yaw)))
        return pose

    def _send_nav(self, pose, purpose):
        goal = NavigateToPose.Goal()
        goal.pose = pose
        self.get_logger().info(
            f'NavigateToPose ({purpose}) -> '
            f'({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f})')
        future = self._nav.send_goal_async(goal)
        future.add_done_callback(
            lambda done: self._on_nav_goal(done, purpose))

    def _on_nav_goal(self, future, purpose):
        handle = future.result()
        if handle is None or not handle.accepted:
            self._fail(f'{purpose} navigation goal rejected')
            return
        handle.get_result_async().add_done_callback(
            lambda done: self._on_nav_result(done, purpose))

    def _on_nav_result(self, future, purpose):
        status = future.result().status
        if status != GoalStatus.STATUS_SUCCEEDED:
            self._fail(f'{purpose} navigation failed (status={status})')
            return
        if purpose == 'plant':
            self.get_logger().info(
                'Nav2 pre-dock reached; starting low-speed odometry-closed '
                'precision docking')
            self._precision_started = time.monotonic()
            self._state = 'precision_docking'
        else:
            self._state = 'done'
            self.get_logger().info(
                'Inspection mission complete: base returned to initial pose.')

    def _base_is_stopped(self):
        if self._latest_odom is None:
            return False
        twist = self._latest_odom.twist.twist
        linear = math.hypot(twist.linear.x, twist.linear.y)
        return (
            linear <= self._linear_limit
            and abs(twist.angular.z) <= self._angular_limit)

    def _selected_plant_pose_in_base(self):
        odom_pose = self._latest_odom.pose.pose
        base_yaw = _quat_to_yaw(odom_pose.orientation)
        dx = float(self._plant['x']) - odom_pose.position.x
        dy = float(self._plant['y']) - odom_pose.position.y
        c = math.cos(base_yaw)
        s = math.sin(base_yaw)
        relative_x = c * dx + s * dy
        relative_y = -s * dx + c * dy
        relative_z = (
            float(self._plant.get('z', 0.0)) - odom_pose.position.z)
        relative_yaw = float(self._plant.get('yaw', 0.0)) - base_yaw
        return relative_x, relative_y, relative_z, relative_yaw

    def _publish_base_speed(self, linear_x=0.0):
        command = Twist()
        command.linear.x = float(linear_x)
        self._cmd_vel.publish(command)

    def _start_pipeline(self):
        x, y, z, yaw = self._selected_plant_pose_in_base()
        self.get_logger().info(
            'Base settled; selected plant in base_footprint: '
            f'x={x:.3f}, y={y:.3f}, z={z:.3f}, yaw={yaw:.3f}. '
            'Starting execute:=true leaf pipeline.')
        try:
            self._result_path.unlink()
        except FileNotFoundError:
            pass
        environment = os.environ.copy()
        environment.update({
            'LEAF_PLANT_POSE': f'{x} {y} {z} 0 0 {yaw}',
            'LEAF_PLANT_FRAME': 'base_footprint',
            'LEAF_PLANT_PROXY_SCALE': '0.5',
            'LEAF_MTC_RESULT_FILE': str(self._result_path),
            'LEAF_MTC_EXIT_AFTER_RESULT': 'true',
        })
        self._pipeline = subprocess.Popen([
            'ros2', 'launch', 'pro450_sim',
            'pro450_myagv_leaf_pipeline.launch.py',
            'execute:=true',
        ], env=environment)
        self._pipeline_started = time.monotonic()
        self._state = 'pipeline'

    def _pipeline_result(self):
        try:
            with self._result_path.open(encoding='utf-8') as stream:
                return json.load(stream)
        except (OSError, ValueError) as error:
            self._fail(f'leaf pipeline did not produce a valid result: {error}')
            return None

    def _arm_is_home(self):
        return (
            all(name in self._arm_positions for name in ARM_JOINTS)
            and max(abs(self._arm_positions[name]) for name in ARM_JOINTS)
            <= self._home_tolerance)

    def _send_arm_home(self):
        if self._home_goal_sent:
            return
        if not self._arm.wait_for_server(timeout_sec=0.1):
            return
        trajectory = JointTrajectory()
        trajectory.joint_names = ARM_JOINTS
        point = JointTrajectoryPoint()
        point.positions = [0.0] * len(ARM_JOINTS)
        point.time_from_start.sec = 5
        trajectory.points = [point]
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        self._home_goal_sent = True
        self.get_logger().warn(
            'MTC reported success but measured arm is not yet home; '
            'sending an explicit home safety command before base return.')
        future = self._arm.send_goal_async(goal)
        future.add_done_callback(self._on_home_goal)

    def _on_home_goal(self, future):
        handle = future.result()
        if handle is None or not handle.accepted:
            self._fail('arm home safety command rejected')
            return
        handle.get_result_async().add_done_callback(self._on_home_result)

    def _on_home_result(self, future):
        result = future.result().result
        if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            self._fail(
                f'arm home safety command failed ({result.error_code})')
            return
        self._state = 'verify_home'

    def _return_base(self):
        self._state = 'returning'
        self._send_nav(self._pose(
            self._initial['x'], self._initial['y'], self._initial['yaw']),
            'home')

    def _fail(self, message):
        if self._state in ('failed', 'done'):
            return
        self._state = 'failed'
        self.get_logger().error(
            f'Inspection mission stopped: {message}. '
            'The base will not return with an unverified arm state.')

    def _tick(self):
        now = time.monotonic()
        if self._state == 'startup':
            if now - self._started_at < self._startup_delay:
                return
            if not self._nav.wait_for_server(timeout_sec=0.1):
                return
            if not self._arm.wait_for_server(timeout_sec=0.1):
                return
            self._state = 'approaching'
            self._send_nav(self._pose(
                self._plant['dock_x'], self._plant['dock_y'],
                self._plant['dock_yaw']), 'plant')
            return

        if self._state == 'settling':
            if self._base_is_stopped():
                if self._settled_since is None:
                    self._settled_since = now
                elif now - self._settled_since >= self._settle_duration:
                    self._start_pipeline()
            else:
                self._settled_since = None
            return

        if self._state == 'precision_docking':
            if self._latest_odom is None:
                return
            x, y, _, _ = self._selected_plant_pose_in_base()
            if abs(y) > 0.12:
                self._publish_base_speed(0.0)
                self._fail(
                    f'precision docking lateral error is unsafe ({y:.3f} m)')
                return
            if x <= self._precision_distance + 0.01:
                self._publish_base_speed(0.0)
                self.get_logger().info(
                    f'Precision dock reached: plant x={x:.3f} m, '
                    f'y={y:.3f} m; verifying continuous zero velocity')
                self._settled_since = None
                self._state = 'settling'
                return
            if now - self._precision_started > self._precision_timeout:
                self._publish_base_speed(0.0)
                self._fail(
                    f'precision docking timeout at plant x={x:.3f} m')
                return
            speed = min(self._precision_speed, max(0.008, 0.4 * (
                x - self._precision_distance)))
            self._publish_base_speed(speed)
            return

        if self._state == 'pipeline':
            if now - self._pipeline_started > self._pipeline_timeout:
                self._pipeline.terminate()
                self._fail('leaf pipeline timeout')
                return
            if self._pipeline.poll() is None:
                return
            result = self._pipeline_result()
            if result is None:
                return
            if not (
                bool(result.get('success'))
                and bool(result.get('executed'))
                and result.get('stage') == 'execution'
            ):
                self._fail(
                    'leaf pipeline was not physically executed successfully: '
                    f'{result}')
                return
            self.get_logger().info(
                'Complete leaf solution executed successfully; '
                'verifying measured arm home state before base return.')
            self._state = 'verify_home'
            self._home_started = now
            return

        if self._state == 'verify_home':
            if self._arm_is_home():
                self.get_logger().info(
                    'Arm home verified from /joint_states; clearing the '
                    'dock-relative MTC scene before returning the base.')
                self._clear_mtc_visualization()
            else:
                if (
                    self._home_started is not None
                    and now - self._home_started > 25.0
                ):
                    self._fail('arm did not reach home within 25 seconds')
                    return
                self._send_arm_home()
            return

        if self._state == 'clear_mtc':
            # Give the reliable transient-local reset time to reach RViz before
            # map -> base_footprint changes again.  Without this hand-off, the
            # retained MTC plant scene appears to travel home with the robot.
            if now - self._mtc_clear_started >= 0.75:
                self._return_base()

    def destroy_node(self):
        self._publish_base_speed(0.0)
        if self._pipeline is not None and self._pipeline.poll() is None:
            self._pipeline.terminate()
        try:
            self._result_path.unlink()
        except FileNotFoundError:
            pass
        return super().destroy_node()


def main():
    rclpy.init()
    node = SinglePlantLeafInspection()
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
