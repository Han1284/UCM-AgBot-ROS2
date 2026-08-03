"""Run one executed inspection mission in an existing environment."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    plants_file = os.path.join(
        get_package_share_directory('pro450_sim'),
        'config', 'atrium_leaf_inspection_plants.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('plant_id'),
        Node(
            package='pro450_sim',
            executable='pro450_myagv_leaf_inspection',
            name='pro450_myagv_leaf_inspection',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'plants_file': plants_file,
                'plant_id': ParameterValue(
                    LaunchConfiguration('plant_id'), value_type=int),
                'execute': True,
                'startup_delay_sec': 0.5,
            }],
        ),
    ])
