"""Atrium 回-corridor Gazebo + myAGV/Pro450 + chassis RGB-D SLAM + RViz.

Terminal workflow:
  1) ros2 launch pro450_sim pro450_myagv_atrium_slam.launch.py
  2) ros2 run pro450_sim pro450_myagv_atrium_slam teleop
  3) ros2 run pro450_sim pro450_myagv_atrium_slam save atrium_map
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
    backend = LaunchConfiguration('backend').perform(context).lower()
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
            '--floor', floor,
            '--atrium', atrium,
            '--model-dir', model_dir,
            '--world-out', world_out,
            '--plant-uri', 'model://simple_potted_plant_pro450',
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

    urdf_path = os.path.join(pkg_description, 'urdf', 'pro450_myagv_pro.urdf.xacro')
    robot_description = {
        'robot_description': ParameterValue(
            Command([
                'xacro ', urdf_path,
                ' use_floor_mount:=false',
                ' use_gz_control:=false',
                ' collision_mesh_scale:=1.0',
                ' include_2d_lidar_mesh:=true',
                ' include_chassis_orbbec:=true',
                ' include_chassis_gz_camera:=false',
                ' include_wrist_camera:=false',
            ]),
            value_type=str,
        ),
        'use_sim_time': True,
    }

    if backend == 'auto':
        backend = 'rtabmap' if _pkg_share('rtabmap_slam') else 'depth_slam'

    actions = [
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resource_path),
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', resource_path),
        LogInfo(msg='[atrium_slam] Gazebo atrium + chassis RGB-D SLAM + RViz walls'),
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
        # Clock + RGB-D + native Gazebo base motion/odometry.  The RGB-D model
        # consumes cmd_vel inside Gazebo, so sensor rendering and odometry share
        # the same simulation update instead of racing external set_pose calls.
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='atrium_slam_bridge',
            output='screen',
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
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='atrium_slam_depth_info_bridge',
            output='screen',
            arguments=[
                '/camera_gz/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            ],
            remappings=[('/camera_gz/camera_info', '/camera/depth/camera_info')],
        ),
        # Gazebo XYZ is X-forward; convert to ROS optical (Z-forward) for SLAM/RViz.
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
            executable='atrium_rviz_markers',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'publish_walls': True,
                'frame_id': 'map',
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
            package='joint_state_publisher',
            executable='joint_state_publisher',
            output='screen',
            parameters=[robot_description],
        ),
        # world ↔ map (identity) so wall markers (frame=map) align with Fixed Frame=map.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_to_map',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--yaw', '0', '--pitch', '0', '--roll', '0',
                '--frame-id', 'world', '--child-frame-id', 'map',
            ],
            parameters=[{'use_sim_time': True}],
        ),
        TimerAction(
            period=8.0,
            actions=[
                # Physics-backed RGB-D base.  The old static full robot was
                # visible to this camera and became a robot-shaped obstacle at
                # the starting pose.  RViz still renders the complete Pro450
                # payload from robot_description and Gazebo TF.
                Node(
                    package='ros_gz_sim',
                    executable='create',
                    name='spawn_chassis_rgbd',
                    output='screen',
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

    # SLAM backend — use optically-corrected cloud (NOT raw Gazebo depth frame).
    # Raw depth stamped/projected as optical caused the 90° map↔cloud mismatch.
    actions.append(Node(
        package='pro450_sim',
        executable='pro450_optical_cloud_to_scan',
        output='screen',
        parameters=[{'use_sim_time': True}],
    ))
    if backend == 'rtabmap' and _pkg_share('rtabmap_slam'):
        actions.append(Node(
            package='rtabmap_slam',
            executable='rtabmap',
            output='screen',
            parameters=[{
                'frame_id': 'base_footprint',
                'odom_frame_id': 'odom',
                'use_sim_time': True,
                'subscribe_depth': False,
                'subscribe_rgb': False,
                'subscribe_scan': True,
                'approx_sync': True,
                'queue_size': 10,
                'Reg/Force3DoF': 'true',
                'Grid/FromDepth': 'false',
                'Grid/3D': 'false',
                'Grid/RangeMax': '8.0',
                'Grid/RayTracing': 'true',
                'Grid/MaxGroundHeight': '0.05',
                'Grid/MaxObstacleHeight': '1.8',
                'Grid/NormalsSegmentation': 'false',
                'RGBD/NeighborLinkRefining': 'true',
                'Mem/STMSize': '30',
            }],
            remappings=[
                ('scan', '/scan'),
            ],
            arguments=['-d'],
        ))
        actions.append(LogInfo(
            msg='[atrium_slam] backend=rtabmap (optical cloud → /scan)'))
    else:
        slam_share = get_package_share_directory('slam_toolbox')
        slam_params = os.path.join(pkg_sim, 'config', 'mapper_params_online_async.yaml')
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(slam_share, 'launch', 'online_async_launch.py')),
            launch_arguments={
                'use_sim_time': 'true',
                'slam_params_file': slam_params,
            }.items(),
        ))
        actions.append(LogInfo(
            msg='[atrium_slam] backend=depth_slam (optical cloud → /scan → slam_toolbox)'))

    if rviz.lower() in ('true', '1'):
        actions.append(Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(pkg_sim, 'rviz', 'pro450_myagv_atrium_slam.rviz')],
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
            'backend', default_value='auto',
            description='auto|rtabmap|depth_slam'),
        DeclareLaunchArgument('x', default_value='-4.0'),
        DeclareLaunchArgument('y', default_value='-4.0'),
        DeclareLaunchArgument('z', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.785'),
        OpaqueFunction(function=_configure),
    ])
