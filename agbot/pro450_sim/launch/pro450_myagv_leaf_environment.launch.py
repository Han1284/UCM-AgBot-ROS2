"""Start the numbered-plant inspection environment without a mission."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    sim_share = get_package_share_directory('pro450_sim')
    return LaunchDescription([
        DeclareLaunchArgument('floor', default_value='concrete'),
        DeclareLaunchArgument('atrium', default_value='stone'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('map', default_value=''),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                sim_share, 'launch',
                'pro450_myagv_leaf_inspection.launch.py')),
            launch_arguments={
                'start_mission': 'false',
                'execute': 'true',
                'floor': LaunchConfiguration('floor'),
                'atrium': LaunchConfiguration('atrium'),
                'gui': LaunchConfiguration('gui'),
                'rviz': LaunchConfiguration('rviz'),
                'map': LaunchConfiguration('map'),
            }.items(),
        ),
    ])
