import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
import yaml


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    with open(absolute_file_path, 'r', encoding='utf-8') as file:
        return yaml.safe_load(file)


def generate_launch_description():
    pkg_sim = get_package_share_directory('leaf_manipulation_sim')

    rviz = LaunchConfiguration('rviz')

    urdf_path = os.path.join(pkg_sim, 'urdf', 'fixed_tm5_rg2.urdf.xacro')
    robot_description = {'robot_description': Command(['xacro ', urdf_path])}
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
    ompl_planning_yaml = load_yaml('tm_moveit_config_tm5-900', 'config/ompl_planning.yaml')
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
    ompl_planning_pipeline_config['ompl'].update(ompl_planning_yaml)

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_to_base_root_tf',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'base_root'],
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[robot_description],
        ),
        Node(
            package='leaf_manipulation_sim',
            executable='static_joint_states',
            name='initial_ready_pose',
            output='screen',
            parameters=[
                {'use_sim_time': False},
                {'publish_rate': 30.0},
                {'duration_sec': 2.0},
                {'joint_names': [
                    'joint_1', 'joint_2', 'joint_3',
                    'joint_4', 'joint_5', 'joint_6', 'finger_joint',
                ]},
                {'joint_positions': [0.0, 0.0, 1.5708, 0.0, 1.5708, 0.0, 0.10]},
            ],
        ),
        Node(
            package='moveit_ros_move_group',
            executable='move_group',
            output='screen',
            parameters=[
                robot_description,
                robot_description_semantic,
                {'robot_description_kinematics': kinematics_yaml},
                {'robot_description_planning': joint_limits_yaml},
                ompl_planning_pipeline_config,
                {'moveit_simple_controller_manager': moveit_controllers},
                {'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager'},
                {'publish_robot_description_semantic': True},
                {'allow_trajectory_execution': False},
            ],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(pkg_sim, 'rviz', 'leaf_manipulation.rviz')],
            condition=IfCondition(rviz),
            parameters=[robot_description, robot_description_semantic],
        ),
    ])
