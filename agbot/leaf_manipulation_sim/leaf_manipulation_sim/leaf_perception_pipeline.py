#!/usr/bin/env python3

"""Run adaptive multi-view perception and globally ranked MTC planning."""

import os
import signal
import subprocess
import sys
import time
from pathlib import Path


def stop_process(process):
    if process is None or process.poll() is not None:
        return
    process.send_signal(signal.SIGINT)
    try:
        process.wait(timeout=5)
    except subprocess.TimeoutExpired:
        process.terminate()


def finalize_candidates():
    """Republish candidates and verify that at least one really exists."""
    try:
        result = subprocess.run(
            [
                'ros2', 'service', 'call',
                '/leaf_perception/finalize',
                'std_srvs/srv/Trigger',
                '{}',
            ],
            check=False,
            capture_output=True,
            text=True,
            timeout=15,
        )
    except subprocess.TimeoutExpired:
        return False, '候选确认服务在 15 秒内没有响应'
    output = '\n'.join(
        part.strip() for part in (result.stdout, result.stderr)
        if part.strip()
    )
    success = (
        result.returncode == 0
        and (
            'success=True' in output
            or 'success=true' in output.lower()
            or 'success: true' in output.lower()
        )
    )
    return success, output


def main():
    virtual_env = os.environ.get('VIRTUAL_ENV')
    perception_python = (
        Path(virtual_env) / 'bin' / 'python3'
        if virtual_env else Path(sys.executable)
    )
    if not perception_python.is_file():
        print(
            f'[leaf_pipeline] 找不到感知 Python: {perception_python}',
            file=sys.stderr,
            flush=True,
        )
        return 2

    print(
        '[leaf_pipeline] 1/4 启动多视角叶片融合与候选点评分节点……',
        flush=True,
    )
    perception = subprocess.Popen([
        str(perception_python),
        '-m', 'leaf_extraction.multi_view_leaf_planner',
        '--ros-args',
        '-p', 'target_frame:=base',
        '-p', 'required_views:=3',
        '-p', 'maximum_views:=5',
    ])
    planner = subprocess.Popen([
        'ros2', 'launch', 'leaf_manipulation_sim',
        'run_leaf_mtc_demo.launch.py',
        'ik_only:=false',
    ])

    print(
        '[leaf_pipeline] 2/4 移动到安全初始观察位，定位冠层并生成斜俯视位……',
        flush=True,
    )
    observation = subprocess.run(
        [
            'ros2', 'launch', 'leaf_manipulation_sim',
            'run_multi_view_observation.launch.py',
            'maximum_views:=5',
        ],
        check=False,
    )
    if observation.returncode != 0:
        print(
            '[leaf_pipeline] 多视角观察未完成。'
            '请查看上方观察位规划、TF 或采集服务错误。',
            file=sys.stderr,
            flush=True,
        )
        stop_process(planner)
        stop_process(perception)
        return observation.returncode

    candidates_ready, finalize_output = finalize_candidates()
    if not candidates_ready:
        print(
            '[leaf_pipeline] 多视角观察结束，但没有形成可发布的表面共识'
            '落点；不会进入空等待的 4/4 阶段。',
            file=sys.stderr,
            flush=True,
        )
        if finalize_output:
            print(finalize_output, file=sys.stderr, flush=True)
        stop_process(planner)
        stop_process(perception)
        return 1

    print(
        '[leaf_pipeline] 3/4 多视角采集完成；正在进行快速 IK、碰撞和'
        '直线预抓取检查……',
        flush=True,
    )
    try:
        print(
            '[leaf_pipeline] 4/4 正在生成完整任务，请不要现在按 Ctrl+C。'
            '看到“Generated N ranked solution(s)”并在 RViz 中确认结果后，'
            '再按 Ctrl+C 结束。',
            flush=True,
        )
        while planner.poll() is None and perception.poll() is None:
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        stop_process(planner)
        stop_process(perception)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
