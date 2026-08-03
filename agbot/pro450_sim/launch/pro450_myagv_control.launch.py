"""myAGV Pro + Pro450 with mock ros2_control and planar /cmd_vel base.

No 3D LiDAR drivers.  Usage:
  ros2 launch pro450_sim pro450_myagv_control.launch.py
  ros2 run pro450_sim pro450_myagv_motion_demo
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    description_pkg = get_package_share_directory('pro450_description')
    sim_pkg = get_package_share_directory('pro450_sim')
    urdf = os.path.join(description_pkg, 'urdf', 'pro450_myagv_pro.urdf.xacro')
    controllers = os.path.join(
        description_pkg, 'config', 'pro450_myagv_controllers.yaml')
    rviz_config = os.path.join(sim_pkg, 'rviz', 'pro450_myagv_control.rviz')

    rviz = LaunchConfiguration('rviz')
    run_demo = LaunchConfiguration('run_demo')

    robot_description = {
        'robot_description': Command([
            'xacro ', urdf,
            ' use_floor_mount:=false',
            ' use_gz_control:=false',
            ' use_fake_hardware:=true',
            ' collision_mesh_scale:=1.0',
            ' include_2d_lidar_mesh:=true',
        ]),
    }

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controllers],
        output='screen',
    )
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description],
        output='screen',
    )
    planar_base = Node(
        package='pro450_sim',
        executable='pro450_myagv_planar_base',
        output='screen',
    )
    jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )
    arm_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_trajectory_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )
    gripper_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_trajectory_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        condition=IfCondition(rviz),
    )
    demo = TimerAction(
        period=6.0,
        actions=[
            Node(
                package='pro450_sim',
                executable='pro450_myagv_motion_demo',
                output='screen',
                condition=IfCondition(run_demo),
            ),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument(
            'run_demo', default_value='true',
            description='Auto-run forward 5m, left turn + 3m, joint2 +90deg'),
        robot_state_publisher,
        controller_manager,
        planar_base,
        jsb,
        RegisterEventHandler(
            OnProcessExit(target_action=jsb, on_exit=[arm_spawner, gripper_spawner])),
        rviz_node,
        demo,
    ])
