"""MoveGroup for the stopped myAGV Pro + Pro450/F100 mobile model."""

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
    use_sim_time = LaunchConfiguration('use_sim_time')
    description_share = get_package_share_directory('pro450_description')
    moveit_share = get_package_share_directory('pro450_moveit_config')
    urdf = os.path.join(
        description_share, 'urdf', 'pro450_myagv_pro.urdf.xacro')
    srdf = os.path.join(
        moveit_share, 'config', 'pro450_myagv_f100.srdf')
    robot_description = {'robot_description': ParameterValue(Command([
        'xacro ', urdf,
        ' use_floor_mount:=false use_gz_control:=false',
        ' collision_mesh_scale:=0.001 planning_fixed_base:=true',
        ' include_2d_lidar_mesh:=false',
        ' include_chassis_orbbec:=false include_chassis_gz_camera:=false',
        ' include_wrist_camera:=true',
    ]), value_type=str)}
    with open(srdf, encoding='utf-8') as stream:
        semantic = {'robot_description_semantic': stream.read()}
    ompl = {'ompl': {
        'planning_plugin': 'ompl_interface/OMPLPlanner',
        'request_adapters': (
            'default_planner_request_adapters/AddTimeOptimalParameterization '
            'default_planner_request_adapters/FixWorkspaceBounds '
            'default_planner_request_adapters/FixStartStateBounds '
            'default_planner_request_adapters/FixStartStateCollision'),
        'start_state_max_bounds_error': 0.1,
    }}
    ompl['ompl'].update(_load_yaml(
        'pro450_moveit_config', 'config/ompl_planning.yaml'))
    parameters = [
        robot_description,
        semantic,
        {'robot_description_planning': _load_yaml(
            'pro450_moveit_config', 'config/joint_limits.yaml')},
        {'robot_description_kinematics': _load_yaml(
            'pro450_moveit_config', 'config/kinematics.yaml')},
        ompl,
        {'planning_pipelines': {
            'pipeline_names': ['ompl'], 'default_planning_pipeline': 'ompl'}},
        _load_yaml('pro450_moveit_config', 'config/moveit_controllers.yaml'),
        {
            'use_sim_time': use_sim_time,
            'publish_robot_description_semantic': True,
            'allow_trajectory_execution': True,
            'capabilities': 'move_group/ExecuteTaskSolutionCapability',
        },
    ]
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        Node(
            package='moveit_ros_move_group',
            executable='move_group',
            name='move_group',
            output='screen',
            parameters=parameters,
        ),
    ])
