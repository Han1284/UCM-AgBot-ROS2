"""Interactive-target atrium inspection with executed Pro450 leaf pinch."""

from __future__ import annotations

import os
import subprocess
import sys
import xml.etree.ElementTree as ET

from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_directory,
)
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
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def _pkg_share(name):
    try:
        return get_package_share_directory(name)
    except PackageNotFoundError:
        return ''


def _mobile_description(urdf_path):
    robot = ET.fromstring(xacro.process_file(urdf_path, mappings={
        'use_floor_mount': 'false',
        'use_gz_control': 'true',
        'use_fake_hardware': 'false',
        'collision_mesh_scale': '1.0',
        'include_2d_lidar_mesh': 'true',
        'include_chassis_orbbec': 'true',
        'include_chassis_gz_camera': 'true',
        'include_wrist_camera': 'true',
    }).toxml())
    for link in robot.findall('link'):
        if link.find('inertial') is not None:
            continue
        inertial = ET.SubElement(link, 'inertial')
        ET.SubElement(inertial, 'mass', value='0.001')
        ET.SubElement(
            inertial, 'inertia',
            ixx='0.000001', ixy='0', ixz='0',
            iyy='0.000001', iyz='0', izz='0.000001')
    return ET.tostring(robot, encoding='unicode')


def _resolve_map(map_arg, sim_share):
    if map_arg:
        if not os.path.isfile(map_arg):
            raise RuntimeError(f'Inspection map does not exist: {map_arg}')
        return map_arg
    output = '/tmp/pro450_atrium_leaf_inspection_map.yaml'
    subprocess.check_call([
        sys.executable,
        os.path.join(sim_share, 'scripts', 'generate_atrium_nav_map.py'),
        '--output-yaml', output,
    ])
    return output


def _configure(context, *args, **kwargs):
    sim_share = get_package_share_directory('pro450_sim')
    description_share = get_package_share_directory('pro450_description')
    moveit_share = get_package_share_directory('pro450_moveit_config')
    ros_gz_share = get_package_share_directory('ros_gz_sim')
    leaf_share = _pkg_share('leaf_manipulation_sim')
    agv_share = _pkg_share('agv_pro_description')
    realsense_share = _pkg_share('realsense2_description')

    floor = LaunchConfiguration('floor').perform(context)
    atrium = LaunchConfiguration('atrium').perform(context)
    gui = LaunchConfiguration('gui').perform(context)
    rviz = LaunchConfiguration('rviz').perform(context).lower() in ('1', 'true')
    execute = LaunchConfiguration('execute').perform(context)
    start_mission = LaunchConfiguration(
        'start_mission').perform(context).lower() in ('1', 'true')
    plant_id = LaunchConfiguration('plant_id').perform(context)
    x = LaunchConfiguration('x').perform(context)
    y = LaunchConfiguration('y').perform(context)
    yaw = LaunchConfiguration('yaw').perform(context)
    map_yaml = _resolve_map(LaunchConfiguration('map').perform(context), sim_share)

    model_dir = os.path.join(sim_share, 'models', 'atrium_corridor_10x10')
    world = os.path.join('/tmp', f'atrium_leaf_inspection_{floor}_{atrium}.sdf')
    subprocess.check_call([
        sys.executable,
        os.path.join(sim_share, 'scripts', 'generate_atrium_corridor_sdf.py'),
        '--floor', floor,
        '--atrium', atrium,
        '--model-dir', model_dir,
        '--world-out', world,
        '--plant-uri', 'model://simple_potted_plant_pro450',
    ])

    urdf = os.path.join(
        description_share, 'urdf', 'pro450_myagv_pro.urdf.xacro')
    robot_xml = _mobile_description(urdf)
    robot_description = {
        'robot_description': robot_xml,
        'use_sim_time': True,
    }
    with open(os.path.join(
            moveit_share, 'config', 'pro450_myagv_f100.srdf'),
            encoding='utf-8') as stream:
        robot_semantic = stream.read()

    resources = os.pathsep.join(filter(None, [
        os.path.join(sim_share, 'models'),
        model_dir,
        os.path.join(leaf_share, 'models') if leaf_share else '',
        os.path.dirname(description_share),
        os.path.dirname(agv_share) if agv_share else '',
        os.path.dirname(realsense_share) if realsense_share else '',
        os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
        os.environ.get('IGN_GAZEBO_RESOURCE_PATH', ''),
    ]))
    nav2_params = os.path.join(
        sim_share, 'config', 'nav2_atrium_params.yaml')
    plants_file = os.path.join(
        sim_share, 'config', 'atrium_leaf_inspection_plants.yaml')
    nav2_launch = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch', 'navigation_launch.py')

    actions = [
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resources),
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', resources),
        LogInfo(msg=(
            '[leaf_inspection] environment starting; '
            f'start_mission={start_mission}; plant={plant_id}; '
            f'execute={execute}')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                ros_gz_share, 'launch', 'gz_sim.launch.py')),
            launch_arguments={
                'gz_args': f'-r {world}' if gui.lower() in ('1', 'true')
                else f'-r -s {world}',
                'gz_version': '6',
            }.items(),
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='leaf_inspection_bridge',
            output='screen',
            arguments=[
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                '/nav_camera_gz/image@sensor_msgs/msg/Image[gz.msgs.Image',
                '/nav_camera_gz/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                '/nav_camera_gz/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
                '/nav_camera_gz/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
                '/wrist_camera_gz/image@sensor_msgs/msg/Image[gz.msgs.Image',
                '/wrist_camera_gz/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                '/wrist_camera_gz/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
                '/wrist_camera_gz/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
                '/model/pro450_myagv/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                '/model/pro450_myagv/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                '/model/pro450_myagv/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            ],
            remappings=[
                ('/nav_camera_gz/image', '/nav_camera/color/image_raw'),
                ('/nav_camera_gz/camera_info', '/nav_camera/color/camera_info'),
                ('/nav_camera_gz/depth_image', '/nav_camera/depth/image_raw'),
                ('/nav_camera_gz/points', '/nav_camera/depth/color/points_gz'),
                ('/wrist_camera_gz/image', '/camera/color/image_raw'),
                ('/wrist_camera_gz/camera_info', '/camera/color/camera_info'),
                ('/wrist_camera_gz/depth_image', '/camera/depth/image_raw'),
                ('/wrist_camera_gz/points', '/camera/depth/color/points_gz'),
                ('/model/pro450_myagv/cmd_vel', '/cmd_vel'),
                ('/model/pro450_myagv/odometry', '/odom'),
                ('/model/pro450_myagv/tf', '/tf'),
            ],
        ),
        Node(
            package='leaf_manipulation_sim',
            executable='point_cloud_optical_adapter',
            name='wrist_cloud_optical_adapter',
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
            package='leaf_manipulation_sim',
            executable='point_cloud_optical_adapter',
            name='nav_cloud_optical_adapter',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'input_topic': '/nav_camera/depth/color/points_gz',
                'output_topic': '/nav_camera/depth/color/points',
                'optical_frame': 'nav_camera_depth_optical_frame',
                'invert_optical_x': True,
                'invert_optical_y': True,
            }],
        ),
        Node(
            package='pro450_sim',
            executable='pro450_optical_cloud_to_scan',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'cloud_topic': '/nav_camera/depth/color/points',
                'scan_topic': '/scan',
                'scan_frame': 'nav_camera_depth_frame',
            }],
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[robot_description],
        ),
        Node(
            package='pro450_sim',
            executable='pro450_robot_description_topic',
            output='screen',
            parameters=[robot_description],
        ),
        Node(
            package='pro450_sim',
            executable='pro450_myagv_wheel_joint_states',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'publish_rate_hz': 20.0,
            }],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_to_map',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--yaw', '0', '--pitch', '0', '--roll', '0',
                '--frame-id', 'world', '--child-frame-id', 'map'],
            parameters=[{'use_sim_time': True}],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--yaw', '0', '--pitch', '0', '--roll', '0',
                '--frame-id', 'map', '--child-frame-id', 'odom'],
            parameters=[{'use_sim_time': True}],
        ),
        Node(
            package='nav2_map_server', executable='map_server',
            name='map_server', output='screen',
            parameters=[{
                'use_sim_time': True,
                'yaml_filename': map_yaml,
                'topic_name': 'map',
                'frame_id': 'map',
            }],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map', output='screen',
            parameters=[{
                'use_sim_time': True,
                'autostart': True,
                'node_names': ['map_server'],
            }],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch),
            launch_arguments={
                'use_sim_time': 'true',
                'autostart': 'true',
                'params_file': nav2_params,
                'use_composition': 'False',
            }.items(),
        ),
        Node(
            package='pro450_sim', executable='atrium_rviz_markers',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'publish_walls': True,
                'publish_collision_boxes': True,
                'publish_plant_labels': True,
                'frame_id': 'map',
            }],
        ),
        TimerAction(period=8.0, actions=[Node(
            package='ros_gz_sim', executable='create',
            name='spawn_pro450_myagv', output='screen',
            arguments=[
                '-world', 'atrium_corridor',
                '-name', 'pro450_myagv',
                '-param', 'robot_description',
                '-x', x, '-y', y, '-z', '0.0', '-Y', yaw,
            ],
            parameters=[{'robot_description': robot_xml}],
        )]),
        # One spawner serializes controller-manager service calls. Running
        # three spawners concurrently can time out on a cold, sensor-heavy
        # Fortress start even though each controller eventually loads.
        TimerAction(period=12.0, actions=[Node(
            package='controller_manager', executable='spawner',
            output='screen',
            arguments=[
                'joint_state_broadcaster',
                'arm_trajectory_controller',
                'gripper_trajectory_controller',
                '--activate-as-group',
                '--controller-manager', '/controller_manager',
                '--controller-manager-timeout', '60.0',
                '--service-call-timeout', '60.0',
                '--switch-timeout', '60.0'],
        )]),
    ]

    if start_mission:
        actions.append(TimerAction(period=22.0, actions=[Node(
            package='pro450_sim',
            executable='pro450_myagv_leaf_inspection',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'plants_file': plants_file,
                'plant_id': int(plant_id),
                'execute': execute.lower() in ('1', 'true'),
                'startup_delay_sec': 2.0,
            }],
        )]))

    if rviz:
        actions.append(Node(
            package='rviz2', executable='rviz2', name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(
                sim_share, 'rviz', 'pro450_fullpipeline.rviz')],
            parameters=[{
                'use_sim_time': True,
                'robot_description': robot_xml,
                'robot_description_semantic': robot_semantic,
            }],
        ))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('plant_id', default_value='1'),
        DeclareLaunchArgument('execute', default_value='true'),
        DeclareLaunchArgument('start_mission', default_value='true'),
        DeclareLaunchArgument('floor', default_value='concrete'),
        DeclareLaunchArgument('atrium', default_value='stone'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('map', default_value=''),
        DeclareLaunchArgument('x', default_value='-4.0'),
        DeclareLaunchArgument('y', default_value='-4.0'),
        DeclareLaunchArgument('yaw', default_value='0.785'),
        OpaqueFunction(function=_configure),
    ])
