"""Run the Pro450 leaf pipeline against an already-running simulation.

Start ``pro450_sim.launch.py`` first.  This launch deliberately owns only
MoveIt, the planning-scene proxies, and the perception/MTC pipeline; it never
starts Gazebo, bridges, robot_state_publisher, or ros2_control a second time.
"""

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
    moveit_share = get_package_share_directory('pro450_moveit_config')
    execute = LaunchConfiguration('execute')
    environment = {
        'LEAF_PLANT_WORLD_PACKAGE': 'pro450_sim',
        'LEAF_PLANT_WORLD_RELATIVE_PATH':
            'worlds/pro450_leaf_bench.world',
        'LEAF_PLANT_FRAME': 'base_root',
        'LEAF_PLANT_PROXY_SCALE': '0.5',
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
            'execute', default_value='false',
            description=(
                'Execute the lowest-cost complete MTC solution in the '
                'already-running Gazebo environment'),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                moveit_share, 'launch', 'pro450_moveit.launch.py')),
            launch_arguments={
                'use_sim_time': 'true',
                'rviz': 'false',
                'joint_state_gui': 'false',
                # The environment launch already owns this node.
                'state_publisher': 'false',
            }.items(),
        ),
        TimerAction(
            # Give the freshly launched MoveGroup time to expose its planning
            # scene service before installing plant collision proxies.
            period=8.0,
            actions=[Node(
                package='leaf_manipulation_sim',
                executable='planning_scene_initializer',
                output='screen',
                additional_env=environment,
                parameters=[{'use_sim_time': True}],
            )],
        ),
        TimerAction(
            period=10.0,
            actions=[pipeline],
        ),
        RegisterEventHandler(OnProcessExit(
            target_action=pipeline,
            on_exit=[EmitEvent(event=Shutdown(
                reason='Pro450 leaf pipeline exited'))],
        )),
    ])
