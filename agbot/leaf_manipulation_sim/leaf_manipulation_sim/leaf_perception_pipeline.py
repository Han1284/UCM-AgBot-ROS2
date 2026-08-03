#!/usr/bin/env python3

"""Run adaptive multi-view perception and globally ranked MTC planning."""

import os
import json
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


def resolve_perception_python() -> Path:
    """Prefer the vision virtualenv when the launcher itself is system Python."""
    virtual_env = os.environ.get('VIRTUAL_ENV')
    candidates = []
    if virtual_env:
        candidates.append(Path(virtual_env) / 'bin' / 'python3')

    script_path = Path(__file__).resolve()
    for parent in script_path.parents:
        candidates.append(parent / '.venv-romu4o' / 'bin' / 'python3')

    candidates.append(Path(sys.executable))
    for candidate in candidates:
        if candidate.is_file():
            return candidate
    return Path(sys.executable)


def call_trigger_service(service_name, timeout_message):
    """Call one Trigger service and retain its human-readable response."""
    try:
        result = subprocess.run(
            [
                'ros2', 'service', 'call',
                service_name,
                'std_srvs/srv/Trigger',
                '{}',
            ],
            check=False,
            capture_output=True,
            text=True,
            timeout=15,
        )
    except subprocess.TimeoutExpired:
        return False, timeout_message
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


def finalize_candidates():
    """Republish candidates and verify that at least one really exists."""
    return call_trigger_service(
        '/leaf_perception/finalize',
        '候选确认服务在 15 秒内没有响应',
    )


def reselect_candidates():
    """Ask perception to reuse fused data and exclude the first shortlist."""
    return call_trigger_service(
        '/leaf_perception/reselect_candidates',
        '一次性差异化换点服务在 15 秒内没有响应',
    )


def read_result(result_path):
    try:
        with open(result_path, encoding='utf-8') as stream:
            return json.load(stream), ''
    except (OSError, ValueError) as error:
        return None, str(error)


def write_attempt_metadata(
    result_path, result, attempt_count, first_attempt_failure=None
):
    """Expose retry provenance to the enclosing mobile mission."""
    if not result_path or result is None:
        return
    result['attempt_count'] = int(attempt_count)
    result['reselected_candidates'] = bool(attempt_count > 1)
    if first_attempt_failure:
        result['first_attempt_failure'] = first_attempt_failure
    try:
        with open(result_path, 'w', encoding='utf-8') as stream:
            json.dump(result, stream, ensure_ascii=False)
    except OSError as error:
        print(
            f'[leaf_pipeline] 警告：无法写入重试元数据：{error}',
            file=sys.stderr,
            flush=True,
        )


def can_retry_mtc(result, pro450, attempt_count):
    """Permit exactly one planning-only recovery, never an execution retry."""
    return bool(
        pro450
        and attempt_count == 1
        and result.get('stage') in ('precheck', 'planning')
        and not bool(result.get('success'))
    )


def main():
    robot_profile = os.environ.get(
        'LEAF_PIPELINE_ROBOT_PROFILE', 'tm5_rg2')
    pro450 = robot_profile == 'pro450_f100'
    launch_package = 'pro450_sim' if pro450 else 'leaf_manipulation_sim'
    mtc_launch = (
        os.environ.get('LEAF_PIPELINE_MTC_LAUNCH', 'pro450_leaf_mtc.launch.py')
        if pro450 else 'run_leaf_mtc_demo.launch.py')
    observation_launch = (
        os.environ.get(
            'LEAF_PIPELINE_OBSERVATION_LAUNCH',
            'pro450_multi_view_observation.launch.py')
        if pro450 else 'run_multi_view_observation.launch.py')
    target_frame = os.environ.get(
        'LEAF_PIPELINE_TARGET_FRAME', 'base_root' if pro450 else 'base')
    point_cloud_topic = os.environ.get(
        'LEAF_PIPELINE_POINT_CLOUD_TOPIC', '/camera/depth/color/points')
    result_file = os.environ.get('LEAF_MTC_RESULT_FILE', '').strip()
    owns_result_file = not bool(result_file)
    if owns_result_file:
        result_file = f'/tmp/leaf_mtc_pipeline_{os.getpid()}.json'
        os.environ['LEAF_MTC_RESULT_FILE'] = result_file
    proxy_surface_gate = os.environ.get(
        'LEAF_PIPELINE_PROXY_SURFACE_GATE', '').strip()
    proxy_keep_fraction = os.environ.get(
        'LEAF_PIPELINE_PROXY_KEEP_FRACTION', '').strip()
    use_proxy_surface_normal = os.environ.get(
        'LEAF_PIPELINE_USE_PROXY_SURFACE_NORMAL', '').strip()
    perception_python = resolve_perception_python()
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
    perception_command = [
        str(perception_python),
        '-m', 'leaf_extraction.multi_view_leaf_planner',
        '--ros-args',
        '-p', f'target_frame:={target_frame}',
        '-p', f'point_cloud_topic:={point_cloud_topic}',
        '-p', f'maximum_views:={"6" if pro450 else "5"}',
    ]
    if pro450:
        perception_command.extend([
            '-p', 'scene_scale:=0.5',
            # Default: initial overview plus one geometry-diverse NBV.
            # Further NBVs are requested adaptively only when the candidate
            # set is insufficient or the fused surface has not converged.
            '-p', 'required_views:=2',
            '-p', 'minimum_leaf_views:=2',
            # Keep budget for adaptive extra NBVs when the candidate set is
            # still thin after the default overview + one validation view.
            '-p', 'minimum_nbv_angular_separation_degrees:=12.0',
            # Distances are scaled by scene_scale=0.5 in the planner.
            '-p', 'minimum_nbv_translation:=0.10',
            # The second geometry-diverse view normally adds about 5-12% new
            # voxels in this half-scale scene.  Treat <=15% as converged so a
            # third NBV is reserved for genuinely incomplete observations.
            '-p', 'low_surface_gain_ratio:=0.15',
            '-p', 'low_surface_gain_patience:=1',
            # Pull the camera farther back so the overview and first NBV
            # actually enclose the whole half-scale canopy, not only the
            # near leaves that happened to fill the previous tight frame.
            '-p', 'overview_extent_scale:=1.60',
            '-p', 'overview_minimum_span:=0.40',
            '-p', 'minimum_downward_pitch_degrees:=20.0',
            '-p', 'minimum_camera_above_canopy:=0.08',
            '-p', 'minimum_view_coverage:=0.85',
            '-p', 'view_frame_margin:=0.10',
            # The planner scales metric parameters by scene_scale=0.5, so
            # this enforces 20 mm between published contacts on one leaf.
            '-p', 'minimum_candidate_separation:=0.04',
            # A recovery shortlist must move at least 30 mm in the half-scale
            # scene relative to every point tried by the first MTC pass.
            '-p', 'retry_candidate_separation:=0.06',
            '-p', 'retry_minimum_candidates:=3',
            # Keep the original Trex outline margin (15% of local half-width).
            # Do not raise this to invent fewer contacts; request more NBV
            # coverage instead when the candidate set is thin.
            '-p', 'minimum_edge_margin_ratio:=0.15',
            # Projection used to clamp near-misses onto the proxy face rim,
            # which published edge contacts that already failed Trex.  A
            # modest inset only restores that rule on the collision face.
            '-p', 'minimum_face_inset_ratio:=0.15',
            # Root->tip entries must deviate by >=45 deg. Tip->root entries
            # remain available when the required insertion fits the F100.
            '-p', 'minimum_root_to_tip_approach_angle_degrees:=45.0',
            '-p', 'gripper_internal_depth:=0.032',
            # Insufficient contacts must trigger more NBV, not a weaker Trex
            # gate.  Four interior contacts across two leaves is the floor.
            '-p', 'minimum_projected_candidates:=4',
            '-p', 'minimum_candidate_leaves:=2',
        ])
        if proxy_surface_gate:
            perception_command.extend([
                '-p', f'maximum_proxy_surface_gate:={proxy_surface_gate}',
            ])
        if proxy_keep_fraction:
            perception_command.extend([
                '-p', f'proxy_surface_keep_fraction:={proxy_keep_fraction}',
            ])
        if use_proxy_surface_normal:
            perception_command.extend([
                '-p', 'use_proxy_surface_normal:='
                f'{use_proxy_surface_normal}',
            ])
    else:
        perception_command.extend(['-p', 'required_views:=3'])
    perception = subprocess.Popen(perception_command)
    planner_command = [
        'ros2', 'launch', launch_package,
        mtc_launch,
        'ik_only:=false',
        'execute:=' + os.environ.get('LEAF_MTC_EXECUTE', 'false'),
    ]
    planner = subprocess.Popen(planner_command)

    print(
        '[leaf_pipeline] 2/4 移动到安全初始观察位，定位冠层并生成斜俯视位……',
        flush=True,
    )
    observation = subprocess.run(
        [
            'ros2', 'launch', launch_package,
            observation_launch,
            f'maximum_views:={"6" if pro450 else "5"}',
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

    print('[leaf_pipeline] 候选确认结果：' + finalize_output, flush=True)
    result = None
    first_attempt_failure = None
    attempt_count = 1
    interrupted = False
    try:
        while True:
            print(
                f'[leaf_pipeline] MTC 规划尝试 {attempt_count}/2：'
                '正在进行裸 IK、碰撞 IK、直线预抓取和完整任务检查。',
                flush=True,
            )
            while planner.poll() is None and perception.poll() is None:
                time.sleep(0.5)
            if perception.poll() is not None:
                print(
                    '[leaf_pipeline] 感知节点意外退出，不能继续使用融合冠层。',
                    file=sys.stderr,
                    flush=True,
                )
                break
            result, result_error = read_result(result_file)
            if result is None:
                print(
                    f'[leaf_pipeline] 第 {attempt_count} 次 MTC 没有产生有效'
                    f'结果文件：{result_error}',
                    file=sys.stderr,
                    flush=True,
                )
                break
            print(
                f'[leaf_pipeline] 第 {attempt_count} 次 MTC 结束：'
                f'成功={bool(result.get("success"))}，'
                f'阶段={result.get("stage", "unknown")}，'
                f'完整解={result.get("solution_count", 0)}，'
                f'说明={result.get("message", "")}。',
                flush=True,
            )
            if bool(result.get('success')):
                break

            if attempt_count == 1:
                first_attempt_failure = {
                    'stage': result.get('stage', 'unknown'),
                    'message': result.get('message', ''),
                    'solution_count': int(result.get('solution_count', 0)),
                }

            retryable_stage = result.get('stage') in ('precheck', 'planning')
            if not can_retry_mtc(result, pro450, attempt_count):
                if not retryable_stage:
                    print(
                        '[leaf_pipeline] 当前失败发生在感知、服务或执行阶段；'
                        '为避免重复动作，不允许自动重试。',
                        file=sys.stderr,
                        flush=True,
                    )
                break

            print(
                '[leaf_pipeline] 首次 MTC 无完整解，启动唯一一次恢复：'
                '先复用现有融合冠层，排除第一次落点附近区域。',
                flush=True,
            )
            reselected, reselect_output = reselect_candidates()
            print(
                '[leaf_pipeline] 第一次差异化换点结果：' + reselect_output,
                flush=True,
            )
            if not reselected:
                print(
                    '[leaf_pipeline] 现有融合云给不出至少 3 个差异化落点；'
                    '追加一轮“概览补充 + 1 个 NBV”，并继续融合原有点云。',
                    flush=True,
                )
                recovery_observation = subprocess.run(
                    [
                        'ros2', 'launch', launch_package,
                        observation_launch,
                        'maximum_views:=2',
                    ],
                    check=False,
                )
                if recovery_observation.returncode != 0:
                    print(
                        '[leaf_pipeline] 额外 NBV 补拍失败；不会继续启动'
                        '第二次 MTC。',
                        file=sys.stderr,
                        flush=True,
                    )
                    break
                reselected, reselect_output = reselect_candidates()
                print(
                    '[leaf_pipeline] 补拍后的差异化换点结果：'
                    + reselect_output,
                    flush=True,
                )
            if not reselected:
                print(
                    '[leaf_pipeline] 补拍后仍没有足够的新落点；停止恢复，'
                    '不会重复使用原落点，也不会进入无限循环。',
                    file=sys.stderr,
                    flush=True,
                )
                break

            try:
                os.unlink(result_file)
            except FileNotFoundError:
                pass
            attempt_count = 2
            print(
                '[leaf_pipeline] 已发布与首次落点保持间距的新候选；'
                '现在启动第二次、也是最后一次 MTC。',
                flush=True,
            )
            planner = subprocess.Popen(planner_command)
    except KeyboardInterrupt:
        interrupted = True
        print('[leaf_pipeline] 收到 Ctrl+C，正在停止管线。', flush=True)
    finally:
        stop_process(planner)
        stop_process(perception)
    if interrupted:
        return 130
    if result is None:
        return 1
    write_attempt_metadata(
        result_file, result, attempt_count, first_attempt_failure)
    if not bool(result.get('success')):
        print(
            '[leaf_pipeline] 抓叶任务最终失败：'
            f'共执行 {attempt_count} 次 MTC，'
            f'最终阶段={result.get("stage", "unknown")}，'
            f'说明={result.get("message", "")}。',
            file=sys.stderr,
            flush=True,
        )
        return 1
    print(
        '[leaf_pipeline] 抓叶任务完成：'
        f'MTC 尝试次数={attempt_count}，'
        f'阶段={result.get("stage")}，'
        f'完整解数量={result.get("solution_count", 0)}。',
        flush=True,
    )
    if owns_result_file:
        try:
            os.unlink(result_file)
        except FileNotFoundError:
            pass
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
