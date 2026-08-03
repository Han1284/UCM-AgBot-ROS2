"""Gazebo Fortress indoor 回-shaped corridor (optional mobile manipulator).

Geometry:
  - Outer footprint: 10 m x 10 m
  - Open atrium: 6 m x 6 m
  - Corridor width: 2 m on all sides
  - No roof; atrium open
  - Outer N/S/E/W doors with closed leaves; atrium walls solid (no doors)
  - Eight simple_potted_plant_pro450 along outer walls (2 per side)

Robot (default on):
  - myAGV Pro + Pro450 / F100 / D435 spawned static in a corridor corner
  - Default pose: SW corner (-4.0, -4.0), yaw=45° toward atrium

Floor options (launch arg floor:=...):
  wood | cherry | concrete | checker | plywood | tarmac | plain

Atrium courtyard color (atrium:=...):
  stone | dark | green

Usage:
  ros2 launch pro450_sim atrium_env.launch.py
  ros2 launch pro450_sim atrium_env.launch.py rviz:=true
  ros2 launch pro450_sim atrium_env.launch.py spawn_robot:=false
  ros2 launch pro450_sim atrium_env.launch.py x:=4.0 y:=4.0 yaw:=-2.356

Plant scale matches Pro450 leaf-bench static test (mesh 0.05); arm/AGV unchanged.
"""

from __future__ import annotations

import os
import subprocess
import sys
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
import xacro


def _rewrite_package_uris(urdf_xml: str) -> str:
    """Turn package:// into absolute file:// for Fortress mesh loading."""
    root = ET.fromstring(urdf_xml)
    cache: dict[str, str] = {}
    for geom in root.iter('mesh'):
        filename = geom.get('filename', '')
        if not filename.startswith('package://'):
            continue
        rest = filename[len('package://'):]
        pkg, _, rel = rest.partition('/')
        if pkg not in cache:
            cache[pkg] = get_package_share_directory(pkg)
        geom.set('filename', 'file://' + os.path.join(cache[pkg], rel))
    return ET.tostring(root, encoding='unicode')


def _gazebo_myagv_description(urdf_path: str) -> str:
    """Static Gazebo entity: fill missing inertias and freeze the model.

    Continuous wheel joints are converted to fixed so Fortress keeps the
    wheel links attached when the whole model is marked static.
    """
    robot = ET.fromstring(xacro.process_file(
        urdf_path,
        mappings={
            'use_floor_mount': 'false',
            'use_gz_control': 'false',
            'use_fake_hardware': 'false',
            'collision_mesh_scale': '1.0',
            'include_2d_lidar_mesh': 'true',
        },
    ).toxml())
    for link in robot.findall('link'):
        if link.find('inertial') is not None:
            continue
        inertial = ET.SubElement(link, 'inertial')
        ET.SubElement(inertial, 'mass', value='0.001')
        ET.SubElement(
            inertial, 'inertia',
            ixx='0.000001', ixy='0', ixz='0',
            iyy='0.000001', iyz='0', izz='0.000001')
    # Static models drop continuous-joint children in Fortress; pin wheels.
    for joint in robot.findall('joint'):
        jtype = joint.get('type', '')
        if jtype in ('continuous', 'revolute', 'prismatic'):
            joint.set('type', 'fixed')
            for tag in ('axis', 'limit', 'dynamics', 'calibration', 'mimic', 'safety_controller'):
                for child in list(joint.findall(tag)):
                    joint.remove(child)
    gazebo = ET.SubElement(robot, 'gazebo')
    ET.SubElement(gazebo, 'static').text = 'true'
    return _rewrite_package_uris(ET.tostring(robot, encoding='unicode'))


def _configure(context, *args, **kwargs):
    pkg_sim = get_package_share_directory('pro450_sim')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_description = get_package_share_directory('pro450_description')
    pkg_agv = get_package_share_directory('agv_pro_description')
    try:
        pkg_leaf = get_package_share_directory('leaf_manipulation_sim')
    except Exception:
        pkg_leaf = ''
    try:
        pkg_realsense = get_package_share_directory('realsense2_description')
    except Exception:
        pkg_realsense = ''

    floor = LaunchConfiguration('floor').perform(context)
    atrium = LaunchConfiguration('atrium').perform(context)
    gui = LaunchConfiguration('gui').perform(context)
    rviz = LaunchConfiguration('rviz').perform(context)
    spawn_robot = LaunchConfiguration('spawn_robot').perform(context)
    x = LaunchConfiguration('x').perform(context)
    y = LaunchConfiguration('y').perform(context)
    z = LaunchConfiguration('z').perform(context)
    yaw = LaunchConfiguration('yaw').perform(context)

    model_dir = os.path.join(pkg_sim, 'models', 'atrium_corridor_10x10')
    gen = os.path.join(pkg_sim, 'scripts', 'generate_atrium_corridor_sdf.py')
    if not os.path.isfile(gen):
        gen = os.path.join(pkg_sim, 'generate_atrium_corridor_sdf.py')

    world_out = os.path.join('/tmp', f'atrium_corridor_{floor}_{atrium}.sdf')
    plant_uri = 'model://simple_potted_plant_pro450'

    if os.path.isfile(gen):
        subprocess.check_call([
            sys.executable, gen,
            '--floor', floor,
            '--atrium', atrium,
            '--model-dir', model_dir,
            '--world-out', world_out,
            '--plant-uri', plant_uri,
        ])
        world = world_out
    else:
        world = os.path.join(pkg_sim, 'worlds', 'atrium_corridor.sdf')

    resource_parts = [
        os.path.join(pkg_sim, 'models'),
        model_dir,
        os.path.join(pkg_leaf, 'models') if pkg_leaf else '',
        os.path.dirname(pkg_description),
        os.path.dirname(pkg_agv),
        os.path.dirname(pkg_realsense) if pkg_realsense else '',
        os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
        os.environ.get('IGN_GAZEBO_RESOURCE_PATH', ''),
    ]
    resource_path = os.pathsep.join([p for p in resource_parts if p])

    actions = [
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resource_path),
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', resource_path),
    ]

    gz_launch = os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
    gz_args = f'-r {world}' if gui.lower() in ('true', '1') else f'-r -s {world}'
    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_launch),
            launch_arguments={'gz_args': gz_args, 'gz_version': '6'}.items(),
        )
    )

    # Clock + RViz markers so Gazebo atrium/plants match RViz (plant scale 0.05).
    actions.append(Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='atrium_clock_bridge',
        output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
    ))
    actions.append(Node(
        package='pro450_sim',
        executable='atrium_rviz_markers',
        output='screen',
        # Wall-clock timers: do not block on /clock (plants must appear in RViz)
        parameters=[{'use_sim_time': False}],
    ))

    if rviz.lower() in ('true', '1'):
        actions.append(Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(pkg_sim, 'rviz', 'atrium_env.rviz')],
            parameters=[{'use_sim_time': True}],
        ))

    if spawn_robot.lower() in ('true', '1'):
        urdf_path = os.path.join(
            pkg_description, 'urdf', 'pro450_myagv_pro.urdf.xacro')
        gazebo_urdf = _gazebo_myagv_description(urdf_path)
        robot_description = Command([
            'xacro ', urdf_path,
            ' use_floor_mount:=false',
            ' use_gz_control:=false',
            ' collision_mesh_scale:=1.0',
            ' include_2d_lidar_mesh:=true',
        ])

        actions.extend([
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                    'robot_description': robot_description,
                }],
            ),
            Node(
                package='joint_state_publisher',
                executable='joint_state_publisher',
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                    'robot_description': robot_description,
                }],
            ),
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='world_to_base_footprint',
                output='screen',
                arguments=[
                    '--x', x, '--y', y, '--z', z,
                    '--yaw', yaw, '--pitch', '0', '--roll', '0',
                    '--frame-id', 'world',
                    '--child-frame-id', 'base_footprint',
                ],
                parameters=[{'use_sim_time': True}],
            ),
            TimerAction(
                period=8.0,
                actions=[Node(
                    package='ros_gz_sim',
                    executable='create',
                    output='screen',
                    arguments=[
                        '-world', 'atrium_corridor',
                        '-name', 'pro450_myagv_pro',
                        '-param', 'robot_description',
                        '-x', x, '-y', y, '-z', z,
                        '-Y', yaw,
                    ],
                    parameters=[{'robot_description': gazebo_urdf}],
                )],
            ),
        ])

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'floor', default_value='concrete',
            description='Corridor floor: wood|cherry|concrete|checker|plywood|tarmac|plain'),
        DeclareLaunchArgument(
            'atrium', default_value='stone',
            description='Atrium courtyard style: stone|dark|green'),
        DeclareLaunchArgument(
            'gui', default_value='true',
            description='Start Gazebo GUI'),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Start RViz with atrium + robot (synced markers)'),
        DeclareLaunchArgument(
            'spawn_robot', default_value='true',
            description='Spawn myAGV Pro + Pro450 in a corridor corner'),
        # SW corridor corner, clear of plants/doors, facing atrium (NE)
        DeclareLaunchArgument('x', default_value='-4.0'),
        DeclareLaunchArgument('y', default_value='-4.0'),
        DeclareLaunchArgument('z', default_value='0.0'),
        DeclareLaunchArgument(
            'yaw', default_value='0.785',
            description='Base yaw (rad); default ~45° toward atrium'),
        OpaqueFunction(function=_configure),
    ])
