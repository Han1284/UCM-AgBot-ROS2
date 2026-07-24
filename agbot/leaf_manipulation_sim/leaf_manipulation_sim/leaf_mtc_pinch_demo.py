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


def gripper_goal(position):
    return {
        'finger_joint': position,
        'left_inner_knuckle_joint': -position,
        'left_inner_finger_joint': position,
        'right_outer_knuckle_joint': -position,
        'right_inner_knuckle_joint': -position,
        'right_inner_finger_joint': position,
    }


def arm_home_goal():
    return {f'joint_{index}': 0.0 for index in range(1, 7)}


def make_task(node):
    """Create the complete approach/pinch/release/return MTC hierarchy."""
    arm_group = 'tmr_arm'
    gripper_group = 'rg2_gripper'
    task = core.Task()
    task.name = 'TM5-900 complete leaf pinch task'
    task.loadRobotModel(node)

    current = stages.CurrentState('1. Current robot state')
    current.timeout = 5.0
    task.add(current)

    joint_planner = core.JointInterpolationPlanner()
    joint_planner.max_velocity_scaling_factor = 0.2
    joint_planner.max_acceleration_scaling_factor = 0.2

    open_gripper = stages.MoveTo('2. Open gripper', joint_planner)
    open_gripper.group = gripper_group
    open_gripper.setGoal(gripper_goal(0.10))
    task.add(open_gripper)

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
        header=Header(frame_id='world'),
        pose=Pose(
            position=Point(x=0.7090, y=-0.2410, z=0.5290),
            orientation=quaternion_from_rpy(
                1.570796,
                -1.543496,
                1.119393,
            ),
        ),
    )

    generator = stages.GeneratePose('4. Leaf 1 segment 5 target pose')
    generator.setMonitoredStage(
        task['2. Open gripper'])
    generator.pose = target

    compute_ik = stages.ComputeIK(
        '5. Solve target pose IK and rank candidates',
        generator,
    )
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

    close_gripper = stages.MoveTo(
        '6. Close to 1 mm clearance per side',
        joint_planner,
    )
    close_gripper.group = gripper_group
    close_gripper.setGoal(gripper_goal(0.680))
    task.add(close_gripper)

    release_gripper = stages.MoveTo('7. Re-open gripper', joint_planner)
    release_gripper.group = gripper_group
    release_gripper.setGoal(gripper_goal(0.10))
    task.add(release_gripper)

    return_planner = core.PipelinePlanner(node)
    return_planner.planner = 'RRTConnect'
    return_planner.max_velocity_scaling_factor = 0.2
    return_planner.max_acceleration_scaling_factor = 0.2
    return_home = stages.MoveTo(
        '8. Return arm to vertical home',
        return_planner,
    )
    return_home.group = arm_group
    return_home.timeout = 1.0
    return_home.setGoal(arm_home_goal())
    task.add(return_home)
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
