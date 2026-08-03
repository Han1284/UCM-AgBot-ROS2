#!/usr/bin/env python3
"""One-shot sanity check: Pro450 is parented under the myAGV Pro TF tree."""

import sys

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener


REQUIRED_FRAMES = (
    'base_footprint',
    'base_link',
    'arm_mount_link',
    'base',
    'link6',
    'gripper_base',
    'camera_link',
)


class Pro450MobileTfCheck(Node):
    def __init__(self):
        super().__init__('pro450_mobile_tf_check')
        self._buffer = Buffer()
        self._listener = TransformListener(self._buffer, self)
        self._timer = self.create_timer(0.5, self._tick)
        self._deadline = self.get_clock().now() + Duration(seconds=15.0)
        self._exit_code = 1

    def _tick(self):
        now = self.get_clock().now()
        if now > self._deadline:
            self.get_logger().error('TF check timed out before all frames appeared')
            self._shutdown()
            return

        missing = [
            frame for frame in REQUIRED_FRAMES
            if not self._buffer.can_transform('base_footprint', frame, rclpy.time.Time())
        ]
        if missing:
            self.get_logger().info(f'Waiting for frames: {", ".join(missing)}')
            return

        try:
            mount = self._buffer.lookup_transform(
                'base_link', 'arm_mount_link', rclpy.time.Time())
            arm = self._buffer.lookup_transform(
                'arm_mount_link', 'base', rclpy.time.Time())
            tip = self._buffer.lookup_transform(
                'base_footprint', 'gripper_base', rclpy.time.Time())
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'TF lookup failed: {exc}')
            self._shutdown()
            return

        mount_z = mount.transform.translation.z
        tip_z = tip.transform.translation.z
        ok = True

        if mount_z < 0.15 or mount_z > 0.28:
            self.get_logger().error(
                f'Arm mount z={mount_z:.4f} m relative to base_link is out of deck range')
            ok = False
        if abs(arm.transform.translation.x) > 1e-6 or abs(arm.transform.translation.z) > 1e-6:
            self.get_logger().error('Pro450 base is not coincident with arm_mount_link')
            ok = False
        if tip_z <= mount_z:
            self.get_logger().error(
                f'gripper_base z={tip_z:.4f} is not above arm mount z={mount_z:.4f}')
            ok = False

        if ok:
            self.get_logger().info(
                'OK: TF tree connects myAGV Pro→Pro450 '
                f'(mount_z={mount_z:.3f}, gripper_z={tip_z:.3f})')
            self._exit_code = 0
        self._shutdown()

    def _shutdown(self):
        self._timer.cancel()
        raise SystemExit(self._exit_code)


def main():
    rclpy.init()
    node = Pro450MobileTfCheck()
    try:
        rclpy.spin(node)
    except SystemExit as exit_exc:
        code = int(exit_exc.code) if exit_exc.code is not None else 1
        try:
            node.destroy_node()
        except Exception:  # noqa: BLE001
            pass
        if rclpy.ok():
            rclpy.shutdown()
        sys.exit(code)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            try:
                node.destroy_node()
            except Exception:  # noqa: BLE001
                pass
            rclpy.shutdown()


if __name__ == '__main__':
    main()
