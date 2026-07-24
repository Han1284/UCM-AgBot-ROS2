#!/usr/bin/env python3

"""Run observation motion, interactive leaf perception, and MTC planning."""

import signal
import subprocess
import sys


def stop_process(process):
    if process is None or process.poll() is not None:
        return
    process.send_signal(signal.SIGINT)
    try:
        process.wait(timeout=5)
    except subprocess.TimeoutExpired:
        process.terminate()


def main():
    print(
        '[leaf_pipeline] 1/3 正在规划并移动到 D435 叶片观察位姿……',
        flush=True,
    )
    observation = subprocess.run(
        [
            'ros2', 'launch', 'leaf_manipulation_sim',
            'run_circle_demo.launch.py',
            'observation_only:=true',
            'hold_final_state:=false',
        ],
        check=False,
    )
    if observation.returncode != 0:
        print(
            '[leaf_pipeline] 观察位姿规划或执行失败，感知未启动。'
            '请查看上方 MoveIt/控制器错误。',
            file=sys.stderr,
            flush=True,
        )
        return observation.returncode

    print(
        '[leaf_pipeline] 2/3 D435 已到观察位姿，正在搜索叶片并等待选择……',
        flush=True,
    )
    planner = subprocess.Popen(
        [
            'ros2', 'launch', 'leaf_manipulation_sim',
            'run_leaf_mtc_demo.launch.py',
            'ik_only:=false',
        ],
    )
    try:
        from leaf_extraction.instance_segmentation import main as perception_main
        perception_main()
    except KeyboardInterrupt:
        pass
    finally:
        stop_process(planner)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
