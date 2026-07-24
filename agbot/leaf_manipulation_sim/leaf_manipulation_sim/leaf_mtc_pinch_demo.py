#!/usr/bin/env python3

"""Publish ranked MoveIt Task Constructor solutions for the leaf pinch pose."""

import math
import os
import signal
import time
from collections import defaultdict

from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from moveit.task_constructor import core, stages
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Header

from custom_interfaces.msg import LeafPoseArrays
from leaf_manipulation_sim.plant_collision_geometry import (
    plant_collision_objects,
)
import rclpy
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


def nearest_leaf_collision_ids(target):
    """Associate a perceived leaf centre with the nearest simulated leaf."""
    grouped = defaultdict(list)
    for collision_object in plant_collision_objects():
        if not collision_object.id.startswith('leaf_'):
            continue
        leaf_name = '_'.join(collision_object.id.split('_')[:2])
        grouped[leaf_name].append(collision_object)

    target_position = target.pose.position
    nearest_name = None
    nearest_distance = float('inf')
    for leaf_name, objects in grouped.items():
        centre_x = sum(
            item.primitive_poses[0].position.x for item in objects
        ) / len(objects)
        centre_y = sum(
            item.primitive_poses[0].position.y for item in objects
        ) / len(objects)
        centre_z = sum(
            item.primitive_poses[0].position.z for item in objects
        ) / len(objects)
        distance = math.sqrt(
            (centre_x - target_position.x) ** 2
            + (centre_y - target_position.y) ** 2
            + (centre_z - target_position.z) ** 2
        )
        if distance < nearest_distance:
            nearest_name = leaf_name
            nearest_distance = distance
    return (
        nearest_name,
        [item.id for item in grouped.get(nearest_name, [])],
        nearest_distance,
    )


def wait_for_selected_leaf():
    """Wait for the interactive perception node to publish one selected leaf."""
    rclpy.init()
    node = rclpy.create_node('leaf_mtc_target_waiter')
    received = []
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

    def callback(message):
        received.append(message)

    subscription = node.create_subscription(
        LeafPoseArrays,
        '/target_leaves_multi_pose',
        callback,
        qos,
    )
    print(
        '[leaf_mtc_pinch_demo] 等待感知节点发布用户选择的叶片……',
        flush=True,
    )
    try:
        while rclpy.ok() and not received:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    del subscription
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    if not received:
        return None

    message = received[-1]
    # Perception publishes a leaf coordinate frame.  The RG2 grasp frame is a
    # separate concept: keep the perceived centre, but use the calibrated
    # TM5/RG2 side-pinch orientation that is reachable for this workcell.
    candidate_sets = (
        message.poses1,
        message.poses2,
        message.poses3,
        message.poses4,
        message.poses5,
    )
    pose = next((poses[0] for poses in candidate_sets if poses), None)
    if pose is None:
        return None
    target = PoseStamped()
    target.header = message.header
    target.pose = pose
    target.pose.orientation = quaternion_from_rpy(
        1.570796,
        -1.543496,
        1.119393,
    )
    # Keep the virtual gripper point slightly before the detected leaf centre.
    # The 35 mm retreat leaves the thin collision boxes between the fingers
    # while keeping the inner knuckles out of the leaf.
    q = target.pose.orientation
    tool_z = (
        2.0 * (q.x * q.z + q.w * q.y),
        2.0 * (q.y * q.z - q.w * q.x),
        1.0 - 2.0 * (q.x * q.x + q.y * q.y),
    )
    fingertip_retreat = 0.035
    target.pose.position.x -= fingertip_retreat * tool_z[0]
    target.pose.position.y -= fingertip_retreat * tool_z[1]
    target.pose.position.z -= fingertip_retreat * tool_z[2]
    return target


def make_task(node, target, ik_only=False):
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

    leaf_name, selected_leaf_collisions, association_distance = (
        nearest_leaf_collision_ids(target))
    print(
        '[leaf_mtc_pinch_demo] 感知目标关联到 '
        f'{leaf_name}，中心距离 {association_distance:.4f} m',
        flush=True,
    )
    fingertip_links = ['left_inner_finger', 'right_inner_finger']
    allow_contact = stages.ModifyPlanningScene(
        '3. Allow selected leaf fingertip contact')
    allow_contact.allowCollisions(
        selected_leaf_collisions,
        fingertip_links,
        True,
    )
    task.add(allow_contact)

    planner = core.PipelinePlanner(node)
    planner.planner = 'RRTConnect'
    planner.max_velocity_scaling_factor = 0.2
    planner.max_acceleration_scaling_factor = 0.2
    connect = stages.Connect(
        '4. Collision-free approach',
        [(arm_group, planner)],
    )
    connect.timeout = 1.0
    task.add(connect)

    generator = stages.GeneratePose('5. Selected perceived leaf target')
    generator.setMonitoredStage(
        task['3. Allow selected leaf fingertip contact'])
    generator.pose = target

    compute_ik = stages.ComputeIK(
        '6. Solve target pose IK and rank candidates',
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

    if ik_only:
        return task

    close_gripper = stages.MoveTo(
        '7. Close to 1 mm clearance per side',
        joint_planner,
    )
    close_gripper.group = gripper_group
    close_gripper.setGoal(gripper_goal(0.680))
    task.add(close_gripper)

    release_gripper = stages.MoveTo('8. Re-open gripper', joint_planner)
    release_gripper.group = gripper_group
    release_gripper.setGoal(gripper_goal(0.10))
    task.add(release_gripper)

    return_planner = core.PipelinePlanner(node)
    return_planner.planner = 'RRTConnect'
    return_planner.max_velocity_scaling_factor = 0.2
    return_planner.max_acceleration_scaling_factor = 0.2
    return_home = stages.MoveTo(
        '9. Return arm to vertical home',
        return_planner,
    )
    return_home.group = arm_group
    return_home.timeout = 1.0
    return_home.setGoal(arm_home_goal())
    task.add(return_home)

    forbid_contact = stages.ModifyPlanningScene(
        '10. Restore selected leaf collision checking')
    forbid_contact.allowCollisions(
        selected_leaf_collisions,
        fingertip_links,
        False,
    )
    task.add(forbid_contact)
    return task


def main():
    target = wait_for_selected_leaf()
    if target is None:
        print(
            '[leaf_mtc_pinch_demo] 未收到有效叶片目标，规划器退出。',
            flush=True,
        )
        return

    print(
        '[leaf_mtc_pinch_demo] 收到目标: '
        f'frame={target.header.frame_id}, '
        f'x={target.pose.position.x:.4f}, '
        f'y={target.pose.position.y:.4f}, '
        f'z={target.pose.position.z:.4f}',
        flush=True,
    )
    rclcpp.init()
    node_options = rclcpp.NodeOptions()
    node_options.automatically_declare_parameters_from_overrides = True
    node = rclcpp.Node('leaf_mtc_pinch_demo', node_options)
    ik_only = os.environ.get(
        'LEAF_MTC_IK_ONLY', 'false').lower() in ('1', 'true', 'yes')
    stop_requested = [False]

    def request_stop(_signum, _frame):
        stop_requested[0] = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    task = None
    try:
        task = make_task(node, target, ik_only=ik_only)
        max_solutions = 12 if ik_only else 5
        result = task.plan(max_solutions)
        if not result:
            print(
                '[leaf_mtc_pinch_demo] First planning attempt returned no '
                'solution; retrying once after ROS discovery settles.',
                flush=True,
            )
            time.sleep(1.0)
            result = task.plan(max_solutions)
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
