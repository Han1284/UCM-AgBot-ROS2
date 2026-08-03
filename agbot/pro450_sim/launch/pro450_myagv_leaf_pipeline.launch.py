"""Run one selected-plant leaf pipeline on an already stopped mobile robot."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    sim_share = get_package_share_directory('pro450_sim')
    execute = LaunchConfiguration('execute')
    environment = {
        'LEAF_PLANT_FRAME': 'base_footprint',
        'LEAF_PLANT_PROXY_SCALE': '0.5',
        'LEAF_PIPELINE_TARGET_FRAME': 'base_footprint',
        'LEAF_PIPELINE_POINT_CLOUD_TOPIC':
            '/camera/depth/color/points',
        # In the atrium the plant stands directly in front of an outer wall.
        # Keep wall pixels leaking through instance masks out of grasp fitting;
        # the complete fused canopy remains published for RViz inspection.
        'LEAF_PIPELINE_PROXY_SURFACE_GATE': '0.065',
        'LEAF_PIPELINE_PROXY_KEEP_FRACTION': '0.25',
        'LEAF_PIPELINE_USE_PROXY_SURFACE_NORMAL': 'true',
        'LEAF_PIPELINE_MTC_LAUNCH':
            'pro450_myagv_leaf_mtc.launch.py',
        'LEAF_PIPELINE_OBSERVATION_LAUNCH':
            'pro450_myagv_multi_view_observation.launch.py',
        'LEAF_MTC_EXIT_AFTER_RESULT': 'true',
    }
    pipeline = Node(
        package='pro450_sim',
        executable='pro450_leaf_pipeline',
        output='screen',
        additional_env={
            **environment,
            'LEAF_MTC_EXECUTE': execute,
        },
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'execute', default_value='true',
            description=(
                'Execute the complete MTC solution. The inspection mission '
                'requires true before the base is allowed to return home.')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                sim_share, 'launch', 'pro450_myagv_moveit.launch.py')),
            launch_arguments={'use_sim_time': 'true'}.items(),
        ),
        TimerAction(
            period=8.0,
            actions=[Node(
                package='leaf_manipulation_sim',
                executable='planning_scene_initializer',
                output='screen',
                additional_env=environment,
                parameters=[{'use_sim_time': True}],
            )],
        ),
        TimerAction(period=10.0, actions=[pipeline]),
        RegisterEventHandler(OnProcessExit(
            target_action=pipeline,
            on_exit=[EmitEvent(event=Shutdown(
                reason='selected-plant mobile leaf pipeline exited'))],
        )),
    ])
