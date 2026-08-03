#!/usr/bin/env python3
"""Run the shared leaf pipeline with the independent Pro450/F100 profile."""

import fcntl
import os
import subprocess


def main():
    lock_handle = open(
        '/tmp/pro450_leaf_pipeline.lock', 'w', encoding='utf-8')
    try:
        fcntl.flock(
            lock_handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        print(
            '[leaf_pipeline] 已有一套 Pro450 叶片管线正在运行；'
            '拒绝启动重复的 MoveGroup/MTC 会话。',
            flush=True,
        )
        lock_handle.close()
        return 2
    lock_handle.write(f'{os.getpid()}\n')
    lock_handle.flush()

    environment = os.environ.copy()
    defaults = {
        'LEAF_PIPELINE_ROBOT_PROFILE': 'pro450_f100',
        'LEAF_MTC_ROBOT_PROFILE': 'pro450_f100',
        'LEAF_PLANT_WORLD_PACKAGE': 'pro450_sim',
        'LEAF_PLANT_WORLD_RELATIVE_PATH': 'worlds/pro450_leaf_bench.world',
        'LEAF_PLANT_FRAME': 'base_root',
        'LEAF_PLANT_PROXY_SCALE': '0.5',
    }
    for name, value in defaults.items():
        environment.setdefault(name, value)
    try:
        return subprocess.call(
            ['ros2', 'run', 'leaf_manipulation_sim',
             'leaf_perception_pipeline'],
            env=environment,
        )
    finally:
        lock_handle.close()


if __name__ == '__main__':
    raise SystemExit(main())
