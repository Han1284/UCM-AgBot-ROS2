"""Gazebo Fortress orchard environment only (no arm, no mobile base).

Uses official osrf/gazebo_models from ~/projects/gazebo_models plus local
wall/vehicle scale wrappers in pro450_sim/models.

Usage:
  ros2 launch pro450_sim orchard_env.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _configure(context, *args, **kwargs):
    pkg_sim = get_package_share_directory('pro450_sim')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    world = LaunchConfiguration('world').perform(context)
    gui = LaunchConfiguration('gui').perform(context)
    gazebo_models = LaunchConfiguration('gazebo_models').perform(context)

    resource_parts = [
        os.path.join(pkg_sim, 'models'),
        gazebo_models,
        os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
        os.environ.get('IGN_GAZEBO_RESOURCE_PATH', ''),
        os.environ.get('GAZEBO_MODEL_PATH', ''),
    ]
    resource_path = os.pathsep.join([p for p in resource_parts if p])

    actions = [
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resource_path),
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', resource_path),
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', resource_path),
    ]

    gz_launch = os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
    if gui.lower() in ('true', '1'):
        gz_args = f'-r {world}'
    else:
        gz_args = f'-r -s {world}'

    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_launch),
            launch_arguments={
                'gz_args': gz_args,
                'gz_version': '6',
            }.items(),
        )
    )
    return actions


def generate_launch_description():
    pkg_sim = get_package_share_directory('pro450_sim')
    default_models = os.environ.get('GAZEBO_MODELS_PATH', '/home/han1284/projects/gazebo_models')

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(pkg_sim, 'worlds', 'orchard_fortress.sdf'),
            description='Fortress orchard world SDF'),
        DeclareLaunchArgument(
            'gui', default_value='true',
            description='Start Gazebo GUI'),
        DeclareLaunchArgument(
            'gazebo_models',
            default_value=default_models,
            description='Path to osrf/gazebo_models checkout'),
        OpaqueFunction(function=_configure),
    ])
