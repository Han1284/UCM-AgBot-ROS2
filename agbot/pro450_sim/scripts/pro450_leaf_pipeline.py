#!/usr/bin/env python3
"""Run the shared leaf pipeline with the independent Pro450/F100 profile."""

import os
import subprocess


def main():
    environment = os.environ.copy()
    environment.update({
        'LEAF_PIPELINE_ROBOT_PROFILE': 'pro450_f100',
        'LEAF_MTC_ROBOT_PROFILE': 'pro450_f100',
        'LEAF_PLANT_WORLD_PACKAGE': 'pro450_sim',
        'LEAF_PLANT_WORLD_RELATIVE_PATH': 'worlds/pro450_leaf_bench.world',
        'LEAF_PLANT_FRAME': 'base_root',
        'LEAF_PLANT_PROXY_SCALE': '0.5',
    })
    return subprocess.call(
        ['ros2', 'run', 'leaf_manipulation_sim', 'leaf_perception_pipeline'],
        env=environment,
    )


if __name__ == '__main__':
    raise SystemExit(main())
