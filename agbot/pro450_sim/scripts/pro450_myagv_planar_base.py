#!/usr/bin/env python3
"""Drive planar odom/TF and keep a Gazebo RGB-D model pose-locked to the robot.

The atrium SLAM stack spawns a non-static `atrium_chassis_rgbd` model.  Moving
that model (not a static full-robot URDF) is what keeps depth aligned with TF
and prevents herringbone / V-trail ghosts in RViz.
"""

from __future__ import annotations

import math
import subprocess
import threading

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class PlanarBaseController(Node):
    def __init__(self):
        super().__init__('pro450_myagv_planar_base')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('rate_hz', 50.0)
        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('initial_yaw', 0.0)

        # Keep a Gazebo model glued to the chassis camera pose.
        self.declare_parameter('sync_gz_pose', False)
        self.declare_parameter('gz_world', 'atrium_corridor')
        self.declare_parameter('gz_model', 'atrium_chassis_rgbd')
        self.declare_parameter('gz_robot_model', 'pro450_myagv_pro')
        self.declare_parameter('gz_sync_hz', 20.0)
        # camera_link in base_footprint: base_joint z=0.02 + camera_joint
        self.declare_parameter('cam_offset_x', 0.23191)
        self.declare_parameter('cam_offset_y', 0.0)
        self.declare_parameter('cam_offset_z', 0.16928)  # 0.02 + 0.14928
        self.declare_parameter('robot_z', 0.0)
        self.declare_parameter('gz_cli', 'gz')  # Fortress: try gz, then ign
        self.declare_parameter('log_gz_failures', True)

        self._odom_frame = self.get_parameter('odom_frame').value
        self._base_frame = self.get_parameter('base_frame').value
        rate_hz = float(self.get_parameter('rate_hz').value)
        self._sync_gz = bool(self.get_parameter('sync_gz_pose').value)
        self._gz_world = str(self.get_parameter('gz_world').value)
        self._gz_model = str(self.get_parameter('gz_model').value)
        self._gz_robot = str(self.get_parameter('gz_robot_model').value)
        self._cam_ox = float(self.get_parameter('cam_offset_x').value)
        self._cam_oy = float(self.get_parameter('cam_offset_y').value)
        self._cam_oz = float(self.get_parameter('cam_offset_z').value)
        self._robot_z = float(self.get_parameter('robot_z').value)
        self._gz_cli = str(self.get_parameter('gz_cli').value)
        self._log_fail = bool(self.get_parameter('log_gz_failures').value)
        gz_sync_hz = max(1.0, float(self.get_parameter('gz_sync_hz').value))
        self._gz_period = 1.0 / gz_sync_hz
        self._gz_last = 0.0
        self._gz_busy = False
        self._gz_lock = threading.Lock()
        self._gz_fail_count = 0
        self._gz_ok_count = 0

        self._x = float(self.get_parameter('initial_x').value)
        self._y = float(self.get_parameter('initial_y').value)
        self._yaw = float(self.get_parameter('initial_yaw').value)
        self._vx = 0.0
        self._vy = 0.0
        self._wz = 0.0
        self._last_time = self.get_clock().now()

        self.create_subscription(Twist, 'cmd_vel', self._on_cmd, 10)
        self._odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self._tf = TransformBroadcaster(self)
        self.create_timer(1.0 / rate_hz, self._tick)
        self.get_logger().info(
            f'Planar base ready: /cmd_vel → {self._odom_frame}→{self._base_frame} '
            f'(start {self._x:.2f},{self._y:.2f},yaw={self._yaw:.3f}, '
            f'sync_gz={self._sync_gz} model={self._gz_model})')

    def _on_cmd(self, msg: Twist):
        self._vx = msg.linear.x
        self._vy = msg.linear.y
        self._wz = msg.angular.z

    def _tick(self):
        now = self.get_clock().now()
        dt = (now - self._last_time).nanoseconds * 1e-9
        self._last_time = now
        if dt <= 0.0 or dt > 0.5:
            return

        cos_y = math.cos(self._yaw)
        sin_y = math.sin(self._yaw)
        self._x += (self._vx * cos_y - self._vy * sin_y) * dt
        self._y += (self._vx * sin_y + self._vy * cos_y) * dt
        self._yaw += self._wz * dt

        qz = math.sin(self._yaw * 0.5)
        qw = math.cos(self._yaw * 0.5)

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self._odom_frame
        odom.child_frame_id = self._base_frame
        odom.pose.pose.position.x = self._x
        odom.pose.pose.position.y = self._y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = self._vx
        odom.twist.twist.linear.y = self._vy
        odom.twist.twist.angular.z = self._wz
        self._odom_pub.publish(odom)

        tf_msg = TransformStamped()
        tf_msg.header.stamp = now.to_msg()
        tf_msg.header.frame_id = self._odom_frame
        tf_msg.child_frame_id = self._base_frame
        tf_msg.transform.translation.x = self._x
        tf_msg.transform.translation.y = self._y
        tf_msg.transform.rotation.z = qz
        tf_msg.transform.rotation.w = qw
        self._tf.sendTransform(tf_msg)

        if self._sync_gz:
            t = now.nanoseconds * 1e-9
            if t - self._gz_last >= self._gz_period:
                self._gz_last = t
                self._fire_gz_camera_pose(qz, qw)

    def _camera_world_pose(self):
        """camera_link pose in world (= odom for this sim)."""
        cos_y = math.cos(self._yaw)
        sin_y = math.sin(self._yaw)
        x = self._x + self._cam_ox * cos_y - self._cam_oy * sin_y
        y = self._y + self._cam_ox * sin_y + self._cam_oy * cos_y
        z = self._cam_oz
        return x, y, z

    def _fire_gz_camera_pose(self, qz: float, qw: float):
        with self._gz_lock:
            if self._gz_busy:
                return
            self._gz_busy = True
        cx, cy, cz = self._camera_world_pose()
        cam_req = (
            f'name: "{self._gz_model}", '
            f'position: {{x: {cx:.4f}, y: {cy:.4f}, z: {cz:.4f}}}, '
            f'orientation: {{x: 0, y: 0, z: {qz:.6f}, w: {qw:.6f}}}'
        )
        robot_req = (
            f'name: "{self._gz_robot}", '
            f'position: {{x: {self._x:.4f}, y: {self._y:.4f}, z: {self._robot_z:.4f}}}, '
            f'orientation: {{x: 0, y: 0, z: {qz:.6f}, w: {qw:.6f}}}'
        )
        service = f'/world/{self._gz_world}/set_pose'

        def _one(req: str) -> tuple[bool, str]:
            err = ''
            for cli in (self._gz_cli, 'gz', 'ign'):
                try:
                    proc = subprocess.run(
                        [
                            cli, 'service',
                            '-s', service,
                            '--reqtype', 'gz.msgs.Pose',
                            '--reptype', 'gz.msgs.Boolean',
                            '--timeout', '300',
                            '--req', req,
                        ],
                        check=False,
                        capture_output=True,
                        text=True,
                    )
                    out = ((proc.stdout or '') + (proc.stderr or '')).lower()
                    if proc.returncode == 0 and (
                            'true' in out or out.strip() == ''):
                        return True, ''
                    err = f'{cli}: rc={proc.returncode} out={out.strip()[:120]}'
                except FileNotFoundError:
                    err = f'{cli}: not found'
                    continue
                except Exception as exc:  # noqa: BLE001
                    err = f'{cli}: {exc}'
            return False, err

        def _run():
            ok_cam, err_cam = _one(cam_req)
            ok_robot, err_robot = _one(robot_req)
            with self._gz_lock:
                self._gz_busy = False
                if ok_cam:
                    self._gz_ok_count += 1
                else:
                    self._gz_fail_count += 1
                    if self._log_fail and self._gz_fail_count <= 5:
                        self.get_logger().warn(
                            f'gz set_pose camera failed: {err_cam}')
                if not ok_robot and self._log_fail and self._gz_fail_count <= 5:
                    self.get_logger().warn(
                        f'gz set_pose robot failed: {err_robot}')

        threading.Thread(target=_run, daemon=True).start()


def main():
    rclpy.init()
    node = PlanarBaseController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:  # noqa: BLE001
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
