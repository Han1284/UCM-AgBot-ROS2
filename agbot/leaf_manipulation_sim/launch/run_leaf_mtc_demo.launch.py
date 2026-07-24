import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
import yaml


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    with open(absolute_file_path, 'r', encoding='utf-8') as file:
        return yaml.safe_load(file)


def generate_launch_description():
    pkg_sim = get_package_share_directory('leaf_manipulation_sim')
    moveit_pkg = get_package_share_directory('tm_moveit_config_tm5-900')

    urdf_path = os.path.join(pkg_sim, 'urdf', 'leaf_arm.sim.urdf.xacro')
    robot_description = {'robot_description': Command(['xacro ', urdf_path])}
    with open(
        os.path.join(moveit_pkg, 'config', 'tm5-900.srdf'),
        encoding='utf-8',
    ) as semantic_file:
        semantic_xml = semantic_file.read()
    semantic_xml = semantic_xml.replace(
        '<virtual_joint name="virtual_joint" type="fixed" '
        'parent_frame="world" child_link="base" />',
        '',
    )
    robot_description_semantic = {
        'robot_description_semantic': semantic_xml
    }
    kinematics_yaml = load_yaml(
        'tm_moveit_config_tm5-900', 'config/kinematics.yaml')
    joint_limits_yaml = load_yaml(
        'tm_moveit_config_tm5-900', 'config/joint_limits.yaml')
    ompl_planning_yaml = load_yaml(
        'tm_moveit_config_tm5-900', 'config/ompl_planning.yaml')
    # The upstream TM5 file leaves this as the literal string "None", which
    # OMPL treats as a missing planner configuration.
    ompl_planning_yaml['tmr_arm']['default_planner_config'] = 'RRTConnect'
    planning_pipeline = {
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
    planning_pipeline['ompl'].update(ompl_planning_yaml)

    return LaunchDescription([
        Node(
            package='leaf_manipulation_sim',
            executable='leaf_mtc_pinch_demo',
            output='screen',
            parameters=[
                robot_description,
                robot_description_semantic,
                {'robot_description_kinematics': kinematics_yaml},
                {'robot_description_planning': joint_limits_yaml},
                planning_pipeline,
                {'use_sim_time': True},
            ],
        ),
    ])
