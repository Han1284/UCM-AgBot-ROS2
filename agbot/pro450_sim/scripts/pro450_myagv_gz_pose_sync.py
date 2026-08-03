#!/usr/bin/env python3
"""Sync planar /odom pose into Gazebo Fortress via `gz service set_pose`.

Keeps the chassis RGB-D camera moving with teleop so SLAM sees the corridor.
"""

from __future__ import annotations

import math
import subprocess
import threading

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


class GzPoseSync(Node):
    def __init__(self):
        super().__init__('pro450_myagv_gz_pose_sync')
        self.declare_parameter('world_name', 'atrium_corridor')
        self.declare_parameter('model_name', 'pro450_myagv_pro')
        self.declare_parameter('odom_topic', 'odom')
        self.declare_parameter('z', 0.0)
        self.declare_parameter('min_period', 0.05)

        self._world = str(self.get_parameter('world_name').value)
        self._model = str(self.get_parameter('model_name').value)
        self._z = float(self.get_parameter('z').value)
        self._min_period = float(self.get_parameter('min_period').value)
        self._last_call = self.get_clock().now()
        self._busy = False
        self._lock = threading.Lock()

        topic = str(self.get_parameter('odom_topic').value)
        self.create_subscription(Odometry, topic, self._on_odom, 10)
        self.get_logger().info(
            f'Syncing /{topic} → gz model "{self._model}" in world "{self._world}"')

    def _on_odom(self, msg: Odometry):
        now = self.get_clock().now()
        if (now - self._last_call).nanoseconds * 1e-9 < self._min_period:
            return
        with self._lock:
            if self._busy:
                return
            self._busy = True
        self._last_call = now

        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        # Prefer yaw-only for planar base (avoid tilt from numerical noise)
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        qz = math.sin(yaw * 0.5)
        qw = math.cos(yaw * 0.5)
        req = (
            f'name: "{self._model}", '
            f'position: {{x: {p.x:.4f}, y: {p.y:.4f}, z: {self._z:.4f}}}, '
            f'orientation: {{x: 0, y: 0, z: {qz:.6f}, w: {qw:.6f}}}'
        )
        service = f'/world/{self._world}/set_pose'
        threading.Thread(
            target=self._call_gz, args=(service, req), daemon=True).start()

    def _call_gz(self, service: str, req: str):
        try:
            subprocess.run(
                [
                    'gz', 'service',
                    '-s', service,
                    '--reqtype', 'gz.msgs.Pose',
                    '--reptype', 'gz.msgs.Boolean',
                    '--timeout', '200',
                    '--req', req,
                ],
                check=False,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
        finally:
            with self._lock:
                self._busy = False


def main():
    rclpy.init()
    node = GzPoseSync()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
