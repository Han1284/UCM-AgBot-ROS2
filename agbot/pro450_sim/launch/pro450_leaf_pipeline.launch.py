"""Complete Pro450/F100 multi-view leaf grasp simulation entry."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    sim_share = get_package_share_directory('pro450_sim')
    moveit_share = get_package_share_directory('pro450_moveit_config')
    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')
    execute = LaunchConfiguration('execute')
    environment = {
        'LEAF_PLANT_WORLD_PACKAGE': 'pro450_sim',
        'LEAF_PLANT_WORLD_RELATIVE_PATH':
            'worlds/pro450_leaf_bench.world',
        'LEAF_PLANT_FRAME': 'base_root',
        'LEAF_PLANT_PROXY_SCALE': '0.5',
    }
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument(
            'execute', default_value='false',
            description=(
                'Execute the best complete MTC solution in Gazebo after '
                'planning; disabled by default for inspection-only runs'),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                sim_share, 'launch', 'pro450_sim.launch.py')),
            launch_arguments={
                'gui': gui,
                'rviz': rviz,
                'control': 'true',
                'use_sim_time': 'true',
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                moveit_share, 'launch', 'pro450_moveit.launch.py')),
            launch_arguments={
                'use_sim_time': 'true',
                'rviz': 'false',
                'joint_state_gui': 'false',
                'state_publisher': 'false',
            }.items(),
        ),
        TimerAction(
            period=10.0,
            actions=[Node(
                package='leaf_manipulation_sim',
                executable='planning_scene_initializer',
                output='screen',
                additional_env=environment,
                parameters=[{'use_sim_time': True}],
            )],
        ),
        TimerAction(
            # The pipeline sends its first arm command immediately.  Let the
            # two command controllers finish activating first.
            period=22.0,
            actions=[Node(
                package='pro450_sim',
                executable='pro450_leaf_pipeline',
                output='screen',
                additional_env={
                    **environment,
                    'LEAF_MTC_EXECUTE': execute,
                },
            )],
        ),
    ])
