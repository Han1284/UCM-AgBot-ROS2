"""MTC leaf planner using the stopped-base arm planning description."""

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
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
    robot_description = {'robot_description': ParameterValue(Command([
        'xacro ', urdf,
        ' use_floor_mount:=false use_gz_control:=false',
        ' collision_mesh_scale:=0.001 planning_fixed_base:=true',
        ' include_2d_lidar_mesh:=false',
        ' include_chassis_orbbec:=false include_chassis_gz_camera:=false',
        ' include_wrist_camera:=true',
    ]), value_type=str)}
    with open(srdf, encoding='utf-8') as stream:
        semantic = stream.read()
    ompl = {'ompl': {
        'planning_plugin': 'ompl_interface/OMPLPlanner',
        'request_adapters': (
            'default_planner_request_adapters/AddTimeOptimalParameterization '
            'default_planner_request_adapters/FixWorkspaceBounds '
            'default_planner_request_adapters/FixStartStateBounds '
            'default_planner_request_adapters/FixStartStateCollision '
            'default_planner_request_adapters/FixStartStatePathConstraints'),
        'start_state_max_bounds_error': 0.1,
    }}
    ompl['ompl'].update(_load_yaml(
        'pro450_moveit_config', 'config/ompl_planning.yaml'))
    environment = {
        'LEAF_MTC_ROBOT_PROFILE': 'pro450_f100',
        'LEAF_MTC_IK_ONLY': LaunchConfiguration('ik_only'),
        'LEAF_MTC_EXECUTE': LaunchConfiguration('execute'),
        'LEAF_MTC_FAST_TIMEOUT_SECONDS':
            LaunchConfiguration('fast_candidate_timeout'),
        'LEAF_MTC_FULL_TIMEOUT_SECONDS':
            LaunchConfiguration('full_task_timeout'),
        'LEAF_MTC_MAX_PROJECTION_WIDTH_RATIO':
            LaunchConfiguration('maximum_projection_width_ratio'),
    }
    parameters = [
        robot_description,
        {'robot_description_semantic': semantic},
        {'robot_description_kinematics': _load_yaml(
            'pro450_moveit_config', 'config/kinematics.yaml')},
        {'robot_description_planning': _load_yaml(
            'pro450_moveit_config', 'config/joint_limits.yaml')},
        ompl,
        {'use_sim_time': True},
    ]
    planner = Node(
        package='leaf_manipulation_sim',
        executable='leaf_mtc_pinch_demo',
        output='screen',
        additional_env=environment,
        parameters=parameters,
    )
    return LaunchDescription([
        DeclareLaunchArgument('ik_only', default_value='false'),
        DeclareLaunchArgument('execute', default_value='false'),
        DeclareLaunchArgument('fast_candidate_timeout', default_value='5.0'),
        DeclareLaunchArgument('full_task_timeout', default_value='20.0'),
        DeclareLaunchArgument(
            'maximum_projection_width_ratio', default_value='1.25'),
        Node(
            package='leaf_manipulation_sim',
            executable='mtc_rviz_task_cleaner',
            output='screen',
        ),
        planner,
        RegisterEventHandler(OnProcessExit(
            target_action=planner,
            on_exit=[EmitEvent(event=Shutdown(
                reason='mobile Pro450 MTC result available'))],
        )),
    ])
