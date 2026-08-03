#!/usr/bin/env python3
"""Project an optical-frame PointCloud2 into a horizontal LaserScan.

Gazebo RGB-D is X-forward; after point_cloud_optical_adapter the cloud is in
camera_depth_optical_frame (Z-forward).  LaserScan itself must be stamped in a
REP-103 planar frame (X-forward, Y-left, Z-up), not in the optical frame.
"""

from __future__ import annotations

import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2


class OpticalCloudToScan(Node):
    def __init__(self):
        super().__init__('pro450_optical_cloud_to_scan')
        self.declare_parameter('cloud_topic', '/camera/depth/color/points')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('scan_frame', 'camera_depth_frame')
        self.declare_parameter('min_height', -0.15)  # optical Y down → negative is up
        self.declare_parameter('max_height', 0.15)
        self.declare_parameter('range_min', 0.2)
        self.declare_parameter('range_max', 8.0)
        self.declare_parameter('angle_min', -1.5708)
        self.declare_parameter('angle_max', 1.5708)
        self.declare_parameter('angle_increment', 0.0087)

        self._ymin = float(self.get_parameter('min_height').value)
        self._ymax = float(self.get_parameter('max_height').value)
        self._rmin = float(self.get_parameter('range_min').value)
        self._rmax = float(self.get_parameter('range_max').value)
        self._amin = float(self.get_parameter('angle_min').value)
        self._amax = float(self.get_parameter('angle_max').value)
        self._ainc = float(self.get_parameter('angle_increment').value)
        self._nbins = int(round((self._amax - self._amin) / self._ainc)) + 1

        cloud_topic = str(self.get_parameter('cloud_topic').value)
        scan_topic = str(self.get_parameter('scan_topic').value)
        self._scan_frame = str(self.get_parameter('scan_frame').value)
        self._pub = self.create_publisher(LaserScan, scan_topic, qos_profile_sensor_data)
        self.create_subscription(
            PointCloud2, cloud_topic, self._on_cloud, qos_profile_sensor_data)
        self.get_logger().info(
            f'Optical cloud {cloud_topic} → LaserScan {scan_topic} '
            f'({self._nbins} beams, frame={self._scan_frame})')

    def _on_cloud(self, msg: PointCloud2):
        offsets = {f.name: f.offset for f in msg.fields}
        if not all(k in offsets for k in ('x', 'y', 'z')):
            return
        n = msg.width * msg.height
        dtype = '>f4' if msg.is_bigendian else '<f4'
        buf = np.frombuffer(msg.data, dtype=np.uint8)

        def col(name):
            return np.ndarray(
                (n,), dtype=dtype, buffer=memoryview(buf),
                offset=offsets[name], strides=(msg.point_step,)).copy()

        x = col('x')  # right
        y = col('y')  # down
        z = col('z')  # forward
        # Horizontal band in optical frame (~constant height in robot frame)
        mask = (
            np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
            & (y >= self._ymin) & (y <= self._ymax)
            & (z > 1e-3))
        x, z = x[mask], z[mask]
        ranges = np.hypot(x, z)
        # In the planar scan frame: X_forward=optical Z and
        # Y_left=-optical X. LaserScan positive angles therefore turn left.
        angles = np.arctan2(-x, z)
        mask2 = (
            (ranges >= self._rmin) & (ranges <= self._rmax)
            & (angles >= self._amin) & (angles <= self._amax))
        ranges, angles = ranges[mask2], angles[mask2]

        bins = np.full(self._nbins, np.inf, dtype=np.float32)
        if ranges.size:
            idx = np.floor((angles - self._amin) / self._ainc).astype(np.int32)
            idx = np.clip(idx, 0, self._nbins - 1)
            # keep nearest return per bin
            order = np.argsort(ranges)
            for i in order:
                b = idx[i]
                if ranges[i] < bins[b]:
                    bins[b] = ranges[i]

        scan = LaserScan()
        scan.header = msg.header
        scan.header.frame_id = self._scan_frame
        scan.angle_min = self._amin
        scan.angle_max = self._amax
        scan.angle_increment = self._ainc
        scan.time_increment = 0.0
        scan.scan_time = 0.1
        scan.range_min = self._rmin
        scan.range_max = self._rmax
        out = bins.tolist()
        scan.ranges = [r if np.isfinite(r) else float('nan') for r in out]
        self._pub.publish(scan)


def main():
    rclpy.init()
    node = OpticalCloudToScan()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
