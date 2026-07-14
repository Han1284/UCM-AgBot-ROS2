import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    with open(absolute_file_path, 'r', encoding='utf-8') as file:
        return yaml.safe_load(file)


def generate_launch_description():
    pkg_sim = get_package_share_directory('leaf_manipulation_sim')
    use_sim_time = LaunchConfiguration('use_sim_time')
    start_move_group = LaunchConfiguration('start_move_group')

    urdf_path = os.path.join(pkg_sim, 'urdf', 'fixed_tm5_rg2.urdf.xacro')
    robot_description = Command(['xacro ', urdf_path])

    robot_description_semantic = {
        'robot_description_semantic': open(
            os.path.join(
                get_package_share_directory('tm_moveit_config_tm5-900'),
                'config',
                'tm5-900.srdf',
            ),
            encoding='utf-8',
        ).read()
    }

    kinematics_yaml = load_yaml('tm_moveit_config_tm5-900', 'config/kinematics.yaml')
    joint_limits_yaml = load_yaml('tm_moveit_config_tm5-900', 'config/joint_limits.yaml')
    moveit_controllers = load_yaml('leaf_manipulation_sim', 'config/moveit_controllers.yaml')

    ompl_planning_pipeline_config = {
        'ompl': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': (
                'default_planner_request_adapters/AddTimeOptimalParameterization '
                'default_planner_request_adapters/FixWorkspaceBounds '
                'default_planner_request_adapters/FixStartStateBounds '
                'default_planner_request_adapters/FixStartStateCollision '
                'default_planner_request_adapters/FixStartStatePathConstraints'
            ),
            'start_state_max_bounds_error': 0.1,
        }
    }
    ompl_planning_yaml = load_yaml('tm_moveit_config_tm5-900', 'config/ompl_planning.yaml')
    ompl_planning_pipeline_config['ompl'].update(ompl_planning_yaml)

    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'robot_description': robot_description},
            robot_description_semantic,
            {'robot_description_kinematics': kinematics_yaml},
            {'robot_description_planning': joint_limits_yaml},
            ompl_planning_pipeline_config,
            {'moveit_simple_controller_manager': moveit_controllers},
            {'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager'},
            {'publish_robot_description_semantic': True},
        ],
        condition=IfCondition(start_move_group),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('start_move_group', default_value='true'),
        move_group_node,
    ])
