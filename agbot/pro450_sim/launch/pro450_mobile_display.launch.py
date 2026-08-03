"""Display myAGV Pro chassis + Pro450 arm only (no 3D LiDAR stacks).

Usage:
  ros2 launch pro450_sim pro450_mobile_display.launch.py

Does NOT start agv_pro_bringup, FAST_LIO, Livox, Unitree, or other lidar drivers.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    description_pkg = get_package_share_directory('pro450_description')
    sim_pkg = get_package_share_directory('pro450_sim')
    urdf = os.path.join(description_pkg, 'urdf', 'pro450_myagv_pro.urdf.xacro')
    rviz_config = os.path.join(sim_pkg, 'rviz', 'pro450_mobile_display.rviz')

    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')
    check_tf = LaunchConfiguration('check_tf')
    include_2d_lidar_mesh = LaunchConfiguration('include_2d_lidar_mesh')

    robot_description = {
        'robot_description': Command([
            'xacro ', urdf,
            ' use_floor_mount:=false',
            ' use_gz_control:=false',
            ' collision_mesh_scale:=1.0',
            ' include_2d_lidar_mesh:=', include_2d_lidar_mesh,
        ]),
    }

    return LaunchDescription([
        DeclareLaunchArgument(
            'gui', default_value='true',
            description='Use joint_state_publisher_gui'),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Start RViz2 with RobotModel + TF'),
        DeclareLaunchArgument(
            'check_tf', default_value='true',
            description='Run a one-shot TF/URDF sanity check after startup'),
        DeclareLaunchArgument(
            'include_2d_lidar_mesh', default_value='true',
            description='Show the stock 2D lidar mesh on the chassis (no driver)'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[robot_description],
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            parameters=[robot_description],
            condition=IfCondition(gui),
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            parameters=[robot_description],
            condition=UnlessCondition(gui),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            condition=IfCondition(rviz),
        ),
        TimerAction(
            period=3.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'run', 'pro450_sim', 'pro450_mobile_tf_check'],
                    output='screen',
                    condition=IfCondition(check_tf),
                ),
            ],
        ),
    ])
