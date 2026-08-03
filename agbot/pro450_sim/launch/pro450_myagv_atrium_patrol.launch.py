"""Atrium Nav2 patrol demo (official-style map + global/local plans).

Loads atrium_map, runs Nav2 (planner + controller), drives one 回 lap with
5s dwell at each plant. Arm stays folded.

  ros2 launch pro450_sim pro450_myagv_atrium_patrol.launch.py
  ros2 run pro450_sim pro450_myagv_atrium_patrol_bringup
"""

from __future__ import annotations

import os
import subprocess
import sys

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _pkg_share(name: str):
    try:
        return get_package_share_directory(name)
    except PackageNotFoundError:
        return None


def _resolve_map(map_arg: str, pkg_sim: str) -> str:
    if map_arg and os.path.isfile(map_arg):
        return map_arg
    if map_arg:
        raise RuntimeError(f'Patrol map does not exist: {map_arg}')

    # Do not default to the SLAM output.  A robot/arm return captured at the
    # initial pose becomes an uncleareable obstacle in Nav2's static layer.
    generator = os.path.join(pkg_sim, 'scripts', 'generate_atrium_nav_map.py')
    output_yaml = '/tmp/pro450_atrium_patrol_map.yaml'
    subprocess.check_call([
        sys.executable, generator, '--output-yaml', output_yaml,
    ])
    return output_yaml


def _configure(context, *args, **kwargs):
    pkg_sim = get_package_share_directory('pro450_sim')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_description = get_package_share_directory('pro450_description')
    pkg_agv = get_package_share_directory('agv_pro_description')
    pkg_leaf = _pkg_share('leaf_manipulation_sim') or ''
    pkg_realsense = _pkg_share('realsense2_description') or ''

    floor = LaunchConfiguration('floor').perform(context)
    atrium = LaunchConfiguration('atrium').perform(context)
    gui = LaunchConfiguration('gui').perform(context)
    rviz = LaunchConfiguration('rviz').perform(context)
    auto_start = LaunchConfiguration('auto_start').perform(context).lower() in (
        'true', '1')
    map_yaml = _resolve_map(LaunchConfiguration('map').perform(context), pkg_sim)
    x = LaunchConfiguration('x').perform(context)
    y = LaunchConfiguration('y').perform(context)
    z = LaunchConfiguration('z').perform(context)
    yaw = LaunchConfiguration('yaw').perform(context)

    model_dir = os.path.join(pkg_sim, 'models', 'atrium_corridor_10x10')
    gen = os.path.join(pkg_sim, 'scripts', 'generate_atrium_corridor_sdf.py')
    if not os.path.isfile(gen):
        gen = os.path.join(pkg_sim, 'generate_atrium_corridor_sdf.py')
    world_out = os.path.join('/tmp', f'atrium_corridor_{floor}_{atrium}.sdf')
    if os.path.isfile(gen):
        subprocess.check_call([
            sys.executable, gen,
            '--floor', floor, '--atrium', atrium,
            '--model-dir', model_dir, '--world-out', world_out,
            '--plant-uri', 'model://simple_potted_plant_pro450',
        ])
        world = world_out
    else:
        world = os.path.join(pkg_sim, 'worlds', 'atrium_corridor.sdf')

    resource_parts = [
        os.path.join(pkg_sim, 'models'), model_dir,
        os.path.join(pkg_leaf, 'models') if pkg_leaf else '',
        os.path.dirname(pkg_description), os.path.dirname(pkg_agv),
        os.path.dirname(pkg_realsense) if pkg_realsense else '',
        os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
        os.environ.get('IGN_GAZEBO_RESOURCE_PATH', ''),
    ]
    resource_path = os.pathsep.join([p for p in resource_parts if p])

    urdf_path = os.path.join(pkg_description, 'urdf', 'pro450_myagv_pro.urdf.xacro')
    robot_description = {
        'robot_description': ParameterValue(
            Command([
                'xacro ', urdf_path,
                ' use_floor_mount:=false use_gz_control:=false',
                ' collision_mesh_scale:=1.0 include_2d_lidar_mesh:=true',
                ' include_chassis_orbbec:=true include_chassis_gz_camera:=false',
                ' include_wrist_camera:=false',
            ]),
            value_type=str,
        ),
        'use_sim_time': True,
    }

    wp_file = os.path.join(pkg_sim, 'config', 'atrium_patrol_waypoints.yaml')
    nav2_params = os.path.join(pkg_sim, 'config', 'nav2_atrium_params.yaml')
    nav2_launch = os.path.join(
        get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')

    actions = [
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resource_path),
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', resource_path),
        LogInfo(msg=f'[atrium_patrol] collision map={map_yaml}'),
        LogInfo(msg='[atrium_patrol] Nav2: map + global/local plan; plant dwells; arm idle'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
            launch_arguments={
                'gz_args': (
                    f'-r {world}' if gui.lower() in ('true', '1')
                    else f'-r -s {world}'),
                'gz_version': '6',
            }.items(),
        ),
        # Clock + RGB-D + native physics-backed base motion/odometry.
        Node(
            package='ros_gz_bridge', executable='parameter_bridge',
            name='atrium_patrol_bridge', output='screen',
            arguments=[
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                '/camera_gz/image@sensor_msgs/msg/Image[gz.msgs.Image',
                '/camera_gz/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                '/camera_gz/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
                '/camera_gz/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
                '/model/atrium_chassis_rgbd/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                '/model/atrium_chassis_rgbd/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                '/model/atrium_chassis_rgbd/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            ],
            remappings=[
                ('/camera_gz/image', '/camera/color/image_raw'),
                ('/camera_gz/camera_info', '/camera/color/camera_info'),
                ('/camera_gz/depth_image', '/camera/depth/image_raw'),
                ('/camera_gz/points', '/camera/depth/color/points_gz'),
                ('/model/atrium_chassis_rgbd/cmd_vel', '/cmd_vel'),
                ('/model/atrium_chassis_rgbd/odometry', '/odom'),
                ('/model/atrium_chassis_rgbd/tf', '/tf'),
            ],
        ),
        Node(
            package='leaf_manipulation_sim',
            executable='point_cloud_optical_adapter',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'input_topic': '/camera/depth/color/points_gz',
                'output_topic': '/camera/depth/color/points',
                'optical_frame': 'camera_depth_optical_frame',
                'invert_optical_x': True,
                'invert_optical_y': True,
            }],
        ),
        Node(
            package='pro450_sim',
            executable='pro450_optical_cloud_to_scan',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),
        Node(
            package='pro450_sim', executable='atrium_rviz_markers', output='screen',
            parameters=[{
                'use_sim_time': False,
                'publish_walls': True,
                'frame_id': 'map',
            }],
        ),
        # Saved map (Transient Local /map for RViz + Nav2 static layer)
        Node(
            package='nav2_map_server', executable='map_server', name='map_server',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'yaml_filename': map_yaml,
                'topic_name': 'map',
                'frame_id': 'map',
            }],
        ),
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_map', output='screen',
            parameters=[{
                'use_sim_time': True,
                'autostart': True,
                'node_names': ['map_server'],
            }],
        ),
        Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            output='screen', parameters=[robot_description],
        ),
        Node(
            package='pro450_sim', executable='pro450_robot_description_topic',
            output='screen', parameters=[robot_description],
        ),
        Node(
            package='joint_state_publisher', executable='joint_state_publisher',
            output='screen', parameters=[robot_description],
        ),
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='world_to_map',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--yaw', '0', '--pitch', '0', '--roll', '0',
                '--frame-id', 'world', '--child-frame-id', 'map',
            ],
            parameters=[{'use_sim_time': True}],
        ),
        # Sim localization: keep map↔odom identity (no AMCL fight)
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='map_to_odom',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--yaw', '0', '--pitch', '0', '--roll', '0',
                '--frame-id', 'map', '--child-frame-id', 'odom',
            ],
            parameters=[{'use_sim_time': True}],
        ),
        # Official Nav2 navigation (global planner + local controller)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch),
            launch_arguments={
                'use_sim_time': 'true',
                'autostart': 'true',
                'params_file': nav2_params,
                'use_composition': 'False',
            }.items(),
        ),
        TimerAction(
            period=8.0,
            actions=[
                Node(
                    package='ros_gz_sim', executable='create',
                    name='spawn_chassis_rgbd', output='screen',
                    arguments=[
                        '-world', 'atrium_corridor',
                        '-name', 'atrium_chassis_rgbd',
                        '-file', os.path.join(
                            pkg_sim, 'models', 'atrium_chassis_rgbd', 'model.sdf'),
                        '-x', x,
                        '-y', y,
                        '-z', '0.0',
                        '-Y', yaw,
                    ],
                ),
            ],
        ),
    ]

    if auto_start:
        actions.append(TimerAction(
            period=20.0,
            actions=[Node(
                package='pro450_sim',
                executable='pro450_myagv_atrium_patrol',
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                    'waypoints_file': wp_file,
                    'startup_delay_sec': 3.0,
                    'default_dwell_sec': 5.0,
                }],
            )],
        ))

    if rviz.lower() in ('true', '1'):
        actions.append(Node(
            package='rviz2', executable='rviz2', name='rviz2', output='screen',
            arguments=['-d', os.path.join(
                pkg_sim, 'rviz', 'pro450_myagv_atrium_patrol.rviz')],
            parameters=[{'use_sim_time': True}],
        ))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('floor', default_value='concrete'),
        DeclareLaunchArgument('atrium', default_value='stone'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument(
            'auto_start', default_value='true',
            description='Start patrol node after Gazebo is up'),
        DeclareLaunchArgument(
            'map', default_value='',
            description=(
                'Optional map YAML. Empty uses the clean deterministic patrol '
                'map; SLAM maps may contain the robot at the start pose.')),
        DeclareLaunchArgument('x', default_value='-4.0'),
        DeclareLaunchArgument('y', default_value='-4.0'),
        DeclareLaunchArgument('z', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.785'),
        OpaqueFunction(function=_configure),
    ])
