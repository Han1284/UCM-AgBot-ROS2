#!/usr/bin/env python3
"""Interactive prompt for triggering the pose-grasp demo."""

from __future__ import annotations

import math
import os
import subprocess
import sys


FIELDS = [
    ('target_x', 'x (m)', 0.30, 0.65, 0.52),
    ('target_y', 'y (m)', -0.35, 0.35, -0.22),
    ('target_z', 'z (m)', 0.12, 0.55, 0.25),
    ('target_roll', 'roll (rad)', -math.pi, math.pi, math.pi),
    ('target_pitch', 'pitch (rad)', -math.pi, math.pi, 0.0),
    ('target_yaw', 'yaw (rad)', -math.pi, math.pi, math.pi / 2.0),
    ('replay_duration_sec', 'motion duration (s)', 1.5, 8.0, 3.0),
]


def ask_float(label: str, lower: float, upper: float, default: float) -> float:
    while True:
        raw = input(f'{label} [{lower:.3f}, {upper:.3f}] default={default:.3f}: ').strip()
        if raw == '':
            return default
        try:
            value = float(raw)
        except ValueError:
            print('请输入数字。')
            continue
        if lower <= value <= upper:
            return value
        print(f'超出建议范围，请输入 {lower:.3f} 到 {upper:.3f} 之间的数。')


def main() -> int:
    print('输入目标末端位姿，单位为米/弧度。')
    print('这是经验可达范围，不保证每个点都一定无碰撞，但通常足够稳定。')

    values: dict[str, float] = {}
    for key, label, lower, upper, default in FIELDS:
        values[key] = ask_float(label, lower, upper, default)

    cmd = [
        'ros2',
        'launch',
        'leaf_manipulation_sim',
        'run_pose_grasp_demo.launch.py',
    ]
    for key, value in values.items():
        cmd.append(f'{key}:={value}')

    env = os.environ.copy()
    env.setdefault('ROS_LOG_DIR', '/tmp/ros_logs')
    print('\n执行目标位姿:')
    print(
        '  x={target_x:.3f}, y={target_y:.3f}, z={target_z:.3f}, '
        'roll={target_roll:.3f}, pitch={target_pitch:.3f}, yaw={target_yaw:.3f}, '
        'duration={replay_duration_sec:.3f}s'.format(**values)
    )
    return subprocess.call(cmd, env=env)


if __name__ == '__main__':
    sys.exit(main())
