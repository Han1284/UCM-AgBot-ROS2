#!/usr/bin/env python3

"""Publish ranked MoveIt Task Constructor solutions for the leaf pinch pose."""

import math
import signal
import time

from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from moveit.task_constructor import core, stages
from std_msgs.msg import Header

import rclcpp


def quaternion_from_rpy(roll, pitch, yaw):
    """Return the same Z-Y-X quaternion convention used by the C++ demo."""
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return Quaternion(
        x=sr * cp * cy - cr * sp * sy,
        y=cr * sp * cy + sr * cp * sy,
        z=cr * cp * sy - sr * sp * cy,
        w=cr * cp * cy + sr * sp * sy,
    )


def make_task(node):
    """Create the official CurrentState/Connect/ComputeIK MTC hierarchy."""
    arm_group = 'tmr_arm'
    task = core.Task()
    task.name = 'TM5-900 leaf pinch candidates'
    task.loadRobotModel(node)

    current = stages.CurrentState('1. Current robot state')
    current.timeout = 5.0
    task.add(current)

    fixed_mount = stages.ModifyPlanningScene(
        '2. Ignore rigid mount overlaps')
    fixed_mount.allowCollisions('link_6', 'camera_link', True)
    fixed_mount.allowCollisions('pedestal_link', 'base', True)
    fixed_mount.allowCollisions('pedestal_link', 'link_0', True)
    task.add(fixed_mount)

    planner = core.PipelinePlanner(node)
    planner.planner = 'RRTConnect'
    planner.max_velocity_scaling_factor = 0.2
    planner.max_acceleration_scaling_factor = 0.2
    connect = stages.Connect(
        '3. Collision-free approach',
        [(arm_group, planner)],
    )
    connect.timeout = 1.0
    task.add(connect)

    target = PoseStamped(
        header=Header(frame_id='base'),
        pose=Pose(
            position=Point(x=0.7093, y=-0.2403, z=0.5064),
            orientation=quaternion_from_rpy(
                1.570796,
                -1.543496,
                1.119393,
            ),
        ),
    )

    generator = stages.GeneratePose('4. Leaf 1 segment 5 target')
    generator.setMonitoredStage(task['2. Ignore rigid mount overlaps'])
    generator.pose = target

    compute_ik = stages.ComputeIK('5. IK candidates and ranking', generator)
    compute_ik.group = arm_group
    compute_ik.ik_frame = PoseStamped(
        header=Header(frame_id='gripper'),
        pose=Pose(orientation=Quaternion(w=1.0)),
    )
    compute_ik.max_ik_solutions = 12
    compute_ik.min_solution_distance = 0.15
    compute_ik.timeout = 0.15
    compute_ik.properties.configureInitFrom(
        core.Stage.PropertyInitializerSource.INTERFACE,
        ['target_pose'],
    )
    task.add(compute_ik)
    return task


def main():
    rclcpp.init()
    node_options = rclcpp.NodeOptions()
    node_options.automatically_declare_parameters_from_overrides = True
    node = rclcpp.Node('leaf_mtc_pinch_demo', node_options)
    stop_requested = [False]

    def request_stop(_signum, _frame):
        stop_requested[0] = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    task = None
    try:
        task = make_task(node)
        result = task.plan(12)
        if not result:
            print(
                '[leaf_mtc_pinch_demo] First planning attempt returned no '
                'solution; retrying once after ROS discovery settles.',
                flush=True,
            )
            time.sleep(1.0)
            result = task.plan(12)
        if not result:
            print(
                '[leaf_mtc_pinch_demo] No complete solution was found. '
                'Inspect failed stages in the Motion Planning Tasks panel.',
                flush=True,
            )
        else:
            solutions = task.solutions
            print(
                f'[leaf_mtc_pinch_demo] Generated {len(solutions)} ranked '
                'solution(s). Select a cost row in RViz to preview it.',
                flush=True,
            )
            task.publish(solutions[0])

        # Keep the introspection services alive while solutions are inspected.
        while not stop_requested[0]:
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        # Ignore repeated launch signals while plugin-owned objects are torn down.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        # Destroy MTC objects before shutting down plugin loaders.
        del task
        del node
        rclcpp.shutdown()


if __name__ == '__main__':
    main()
