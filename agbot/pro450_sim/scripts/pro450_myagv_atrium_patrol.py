#!/usr/bin/env python3
"""Nav2 plant-circuit patrol: one 回 lap, 5s dwell at each plant (arm idle).

Sends nav2_msgs/action/NavigateToPose goals so RViz shows Global / Local Plan.
"""

from __future__ import annotations

import math
from pathlib import Path

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray


def _yaw_to_quat(yaw: float):
    return 0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5)


class AtriumNav2Patrol(Node):
    def __init__(self):
        super().__init__('pro450_myagv_atrium_patrol')
        self.declare_parameter('waypoints_file', '')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('default_dwell_sec', 5.0)
        self.declare_parameter('startup_delay_sec', 18.0)
        self.declare_parameter('stop_on_failure', True)

        path = str(self.get_parameter('waypoints_file').value)
        if not path or not Path(path).is_file():
            raise RuntimeError(f'waypoints_file missing: {path}')
        with open(path, 'r', encoding='utf-8') as f:
            cfg = yaml.safe_load(f)

        self._map_frame = str(self.get_parameter('map_frame').value)
        self._default_dwell = float(self.get_parameter('default_dwell_sec').value)
        self._start_delay = float(self.get_parameter('startup_delay_sec').value)
        self._stop_on_failure = bool(
            self.get_parameter('stop_on_failure').value)

        self._waypoints = []
        for phase in cfg.get('phases', []):
            for wp in phase.get('waypoints', []):
                self._waypoints.append({
                    'phase': phase.get('name', ''),
                    'name': wp.get('name', 'wp'),
                    'x': float(wp['x']),
                    'y': float(wp['y']),
                    'yaw': float(wp['yaw']),
                    'dwell': float(wp.get(
                        'dwell', cfg.get('default_dwell_sec', self._default_dwell))),
                })

        self._nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._mark_pub = self.create_publisher(MarkerArray, '/atrium_patrol_markers', 10)
        self._idx = 0
        self._state = 'wait'  # wait|navigating|dwell|done
        self._dwell_left = 0.0
        self._elapsed = 0.0
        self.create_timer(0.2, self._tick)
        self.create_timer(1.0, self._publish_markers)
        self.get_logger().info(
            f'Nav2 patrol: {len(self._waypoints)} waypoints, '
            f'start in {self._start_delay:.0f}s (arm idle)')

    def _publish_markers(self):
        arr = MarkerArray()
        for i, wp in enumerate(self._waypoints):
            m = Marker()
            m.header.frame_id = self._map_frame
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'patrol_docks'
            m.id = i
            m.type = Marker.ARROW
            m.action = Marker.ADD
            m.pose.position.x = wp['x']
            m.pose.position.y = wp['y']
            m.pose.position.z = 0.15
            _, _, z, w = _yaw_to_quat(wp['yaw'])
            m.pose.orientation.z = z
            m.pose.orientation.w = w
            m.scale.x, m.scale.y, m.scale.z = 0.45, 0.08, 0.08
            if i == self._idx and self._state != 'done':
                m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.45, 0.1, 0.95
            elif i < self._idx:
                m.color.r, m.color.g, m.color.b, m.color.a = 0.2, 0.8, 0.3, 0.7
            else:
                m.color.r, m.color.g, m.color.b, m.color.a = 0.2, 0.5, 1.0, 0.7
            arr.markers.append(m)
        self._mark_pub.publish(arr)

    def _tick(self):
        dt = 0.2
        self._elapsed += dt

        if self._state == 'done':
            return

        if self._state == 'dwell':
            self._dwell_left -= dt
            if self._dwell_left <= 0.0:
                self._idx += 1
                self._send_current()
            return

        if self._state == 'wait':
            if self._elapsed < self._start_delay:
                return
            if not self._nav.wait_for_server(timeout_sec=0.1):
                if int(self._elapsed * 5) % 25 == 0:
                    self.get_logger().warn('Waiting for navigate_to_pose...')
                return
            self.get_logger().info('Nav2 ready — starting 回 circuit')
            self._send_current()

    def _send_current(self):
        if self._idx >= len(self._waypoints):
            self._state = 'done'
            self.get_logger().info('Patrol complete (one 回 lap with plant dwells).')
            return

        wp = self._waypoints[self._idx]
        pose = PoseStamped()
        pose.header.frame_id = self._map_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = wp['x']
        pose.pose.position.y = wp['y']
        _, _, z, w = _yaw_to_quat(wp['yaw'])
        pose.pose.orientation.z = z
        pose.pose.orientation.w = w

        goal = NavigateToPose.Goal()
        goal.pose = pose
        self._state = 'navigating'
        self.get_logger().info(
            f'[{wp["phase"]}] NavigateToPose → {wp["name"]} '
            f'({wp["x"]:.2f},{wp["y"]:.2f})')
        send_future = self._nav.send_goal_async(goal)
        send_future.add_done_callback(self._goal_response)

    def _goal_response(self, future):
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error('Goal rejected — skip')
            self._idx += 1
            self._send_current()
            return
        goal_handle.get_result_async().add_done_callback(self._goal_result)

    def _goal_result(self, future):
        status = future.result().status
        wp = self._waypoints[self._idx]
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'Reached {wp["name"]}')
        else:
            self.get_logger().error(
                f'{wp["name"]} navigation failed (status={status}); '
                'the waypoint was not reached, so no inspection dwell is run')
            if self._stop_on_failure:
                self._state = 'done'
                self.get_logger().error(
                    'Patrol stopped. Check the global costmap and waypoint '
                    'clearance before retrying.')
                return
            self._idx += 1
            self._send_current()
            return

        dwell = wp['dwell'] if wp['dwell'] is not None else self._default_dwell
        if dwell > 0.0:
            self.get_logger().info(f'Dwell {dwell:.1f}s (arm idle)')
            self._dwell_left = dwell
            self._state = 'dwell'
        else:
            self._idx += 1
            self._send_current()


def main():
    rclpy.init()
    node = AtriumNav2Patrol()
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
