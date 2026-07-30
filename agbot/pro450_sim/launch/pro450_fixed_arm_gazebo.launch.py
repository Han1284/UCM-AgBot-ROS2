import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui')
    leaf_pkg = get_package_share_directory('leaf_manipulation_sim')
    description_pkg = get_package_share_directory('pro450_description')
    sim_pkg = get_package_share_directory('pro450_sim')
    gazebo_pkg = get_package_share_directory('gazebo_ros')
    urdf = os.path.join(description_pkg, 'urdf', 'pro450_f100.urdf.xacro')
    robot_description = {'robot_description': Command(['xacro ', urdf])}
    world = os.path.join(leaf_pkg, 'worlds', 'fixed_arm_empty.world')
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_pkg, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': world, 'use_sim_time': use_sim_time}.items())
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_pkg, 'launch', 'gzclient.launch.py')),
        condition=IfCondition(gui))
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true'),
        SetEnvironmentVariable(name='GAZEBO_RESOURCE_PATH', value=':'.join(['/usr/share/gazebo-11', '/usr/share/gazebo-11/media', description_pkg, leaf_pkg, os.environ.get('GAZEBO_RESOURCE_PATH', '')])),
        SetEnvironmentVariable(name='GAZEBO_MODEL_PATH', value=':'.join([os.path.join(leaf_pkg, 'models'), '/usr/share/gazebo-11/models', os.environ.get('GAZEBO_MODEL_PATH', '')])),
        SetEnvironmentVariable(name='GAZEBO_PLUGIN_PATH', value=':'.join(['/usr/lib/x86_64-linux-gnu/gazebo-11/plugins', os.environ.get('GAZEBO_PLUGIN_PATH', '')])),
        SetEnvironmentVariable(name='GAZEBO_MODEL_DATABASE_URI', value=''),
        gzserver,
        gzclient,
        Node(package='robot_state_publisher', executable='robot_state_publisher', parameters=[robot_description, {'use_sim_time': use_sim_time}]),
        Node(package='pro450_sim', executable='pro450_static_joint_states', parameters=[{'use_sim_time': use_sim_time}]),
        TimerAction(period=3.0, actions=[Node(package='gazebo_ros', executable='spawn_entity.py', arguments=['-entity', 'pro450_f100', '-topic', 'robot_description', '-x', '0.0', '-y', '0.0', '-z', '0.0'], output='screen')]),
        Node(package='leaf_manipulation_sim', executable='plant_marker_publisher', parameters=[{'use_sim_time': use_sim_time}], condition=IfCondition(gui)),
    ])
