import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def _load_yaml(package, relative_path):
    with open(os.path.join(get_package_share_directory(package), relative_path), encoding='utf-8') as stream:
        return yaml.safe_load(stream)


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz = LaunchConfiguration('rviz')
    joint_state_gui = LaunchConfiguration('joint_state_gui')
    state_publisher = LaunchConfiguration('state_publisher')
    description_pkg = get_package_share_directory('pro450_description')
    config_pkg = get_package_share_directory('pro450_moveit_config')
    urdf = os.path.join(description_pkg, 'urdf', 'pro450_f100.urdf.xacro')
    robot_description = {'robot_description': Command([
        'xacro ', urdf, ' collision_mesh_scale:=0.001'])}
    semantic = {'robot_description_semantic': open(os.path.join(config_pkg, 'config', 'pro450_f100.srdf'), encoding='utf-8').read()}
    planning = {'robot_description_planning': _load_yaml('pro450_moveit_config', 'config/joint_limits.yaml')}
    kinematics = {'robot_description_kinematics': _load_yaml('pro450_moveit_config', 'config/kinematics.yaml')}
    ompl = {'ompl': {'planning_plugin': 'ompl_interface/OMPLPlanner', 'request_adapters': 'default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision', 'start_state_max_bounds_error': 0.1}}
    ompl['ompl'].update(_load_yaml('pro450_moveit_config', 'config/ompl_planning.yaml'))
    controllers = _load_yaml('pro450_moveit_config', 'config/moveit_controllers.yaml')
    pipelines = {'planning_pipelines': {'pipeline_names': ['ompl'], 'default_planning_pipeline': 'ompl'}}
    execution = {
        'allow_trajectory_execution': True,
        'capabilities': 'move_group/ExecuteTaskSolutionCapability',
    }
    parameters = [robot_description, semantic, planning, kinematics, ompl, pipelines, controllers, {'use_sim_time': use_sim_time, 'publish_robot_description_semantic': True}, execution]
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('joint_state_gui', default_value='true'),
        DeclareLaunchArgument('state_publisher', default_value='true'),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            parameters=[robot_description],
            condition=IfCondition(joint_state_gui)),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[robot_description, {'use_sim_time': use_sim_time}],
            condition=IfCondition(state_publisher)),
        Node(package='moveit_ros_move_group', executable='move_group', output='screen', parameters=parameters),
        Node(package='rviz2', executable='rviz2', output='screen', arguments=['-d', os.path.join(config_pkg, 'config', 'moveit.rviz')], parameters=parameters, condition=IfCondition(rviz)),
    ])
