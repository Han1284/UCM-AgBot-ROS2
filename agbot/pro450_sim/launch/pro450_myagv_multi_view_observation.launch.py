"""Adaptive wrist-camera observations on the stopped mobile platform."""

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _load_yaml(package, relative_path):
    with open(os.path.join(
            get_package_share_directory(package), relative_path),
            encoding='utf-8') as stream:
        return yaml.safe_load(stream)


def generate_launch_description():
    description_share = get_package_share_directory('pro450_description')
    moveit_share = get_package_share_directory('pro450_moveit_config')
    urdf = os.path.join(
        description_share, 'urdf', 'pro450_myagv_pro.urdf.xacro')
    srdf = os.path.join(
        moveit_share, 'config', 'pro450_myagv_f100.srdf')
    with open(srdf, encoding='utf-8') as stream:
        semantic = stream.read()
    return LaunchDescription([
        DeclareLaunchArgument('maximum_views', default_value='6'),
        DeclareLaunchArgument('settle_seconds', default_value='0.45'),
        Node(
            package='leaf_manipulation_sim',
            executable='multi_view_observation_demo',
            output='screen',
            parameters=[
                {'robot_description': ParameterValue(Command([
                    'xacro ', urdf,
                    ' use_floor_mount:=false use_gz_control:=false',
                    ' collision_mesh_scale:=0.001 planning_fixed_base:=true',
                    ' include_2d_lidar_mesh:=false',
                    ' include_chassis_orbbec:=false',
                    ' include_chassis_gz_camera:=false',
                    ' include_wrist_camera:=true',
                ]), value_type=str)},
                {'robot_description_semantic': semantic},
                {'robot_description_kinematics': _load_yaml(
                    'pro450_moveit_config', 'config/kinematics.yaml')},
                {
                    'arm_group': 'pro450_arm',
                    'base_frame': 'base_footprint',
                    'end_effector_link': 'gripper_base',
                    'camera_optical_frame': 'camera_depth_optical_frame',
                    'arm_joint_names': [
                        'joint1', 'joint2', 'joint3',
                        'joint4', 'joint5', 'joint6'],
                    'execute_via_move_group': True,
                    'initial_observation_joint_positions':
                        [0.0, 0.4, -1.2, 0.0, 0.0, 0.4],
                    'maximum_views': LaunchConfiguration('maximum_views'),
                    'settle_seconds': LaunchConfiguration('settle_seconds'),
                    'use_sim_time': True,
                },
            ],
        ),
    ])
