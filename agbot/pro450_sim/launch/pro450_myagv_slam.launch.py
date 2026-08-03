"""myAGV Pro SLAM using onboard Orbbec 3D camera (no fake laser).

Starts by default:
  1) agv_pro_node (real odom / cmd_vel)
  2) Full mobile manipulator model in RViz (chassis + Pro450, arm held at home)
  3) Orbbec Gemini2
  4) RTAB-Map or depth→slam_toolbox

Camera topics (Orbbec Gemini2 / vendor default):
  /camera/color/image_raw
  /camera/color/camera_info
  /camera/depth/image_raw
  /camera/depth/camera_info

Usage:
  ros2 launch pro450_sim pro450_myagv_slam.launch.py
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _pkg_share(name: str):
    try:
        return get_package_share_directory(name)
    except PackageNotFoundError:
        return None


def _orbbec_gemini2_include():
    share = _pkg_share('orbbec_camera')
    if share:
        path = os.path.join(share, 'launch', 'gemini2.launch.py')
        if os.path.isfile(path):
            return IncludeLaunchDescription(PythonLaunchDescriptionSource(path))
    src = os.path.expanduser(
        '~/projects/ros2_ws/src/agv_pro_ros2/OrbbecSDK_ROS2/'
        'orbbec_camera/launch/gemini2.launch.py')
    if os.path.isfile(src):
        return IncludeLaunchDescription(PythonLaunchDescriptionSource(src))
    return None


def _agv_bringup_include(port_name: str, enable_lidar: str, lidar_type: str):
    share = _pkg_share('agv_pro_bringup')
    if not share:
        return None
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(share, 'launch', 'agv_pro_bringup.launch.py')),
        launch_arguments={
            'port_name': port_name,
            'enable_lidar': enable_lidar,
            'lidar_type': lidar_type,
        }.items(),
    )


def _mobile_manipulator_nodes(port_name: str):
    """Real base driver + full chassis+arm TF (arm joints held at zero/home)."""
    desc_share = _pkg_share('pro450_description')
    if not desc_share:
        return None, '[slam] pro450_description not found — cannot show manipulator'

    urdf = os.path.join(desc_share, 'urdf', 'pro450_myagv_pro.urdf.xacro')
    robot_description = {
        'robot_description': ParameterValue(
            Command([
                'xacro ', urdf,
                ' use_floor_mount:=false',
                ' use_gz_control:=false',
                ' use_fake_hardware:=false',
                ' collision_mesh_scale:=1.0',
                ' include_2d_lidar_mesh:=true',
                ' include_chassis_orbbec:=true',
                ' include_wrist_camera:=false',
            ]),
            value_type=str,
        ),
    }

    nodes = [
        Node(
            package='agv_pro_base',
            executable='agv_pro_node',
            name='agv_pro_node',
            output='screen',
            parameters=[{'port_name': port_name, 'namespace': ''}],
            remappings=[('cmd_vel', '/cmd_vel')],
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[robot_description],
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[robot_description],
        ),
    ]
    return nodes, (
        '[slam] base=agv_pro_node + RobotModel=myAGV Pro + Pro450 '
        '(arm held at home; teleop only drives the base)'
    )


def _configure(context, *args, **kwargs):
    backend = LaunchConfiguration('backend').perform(context).lower()
    use_sim = LaunchConfiguration('use_sim_time').perform(context).lower() in ('true', '1')
    start_bringup = LaunchConfiguration('start_bringup').perform(context).lower() in ('true', '1')
    start_camera = LaunchConfiguration('start_camera').perform(context).lower() in ('true', '1')
    show_manipulator = LaunchConfiguration('show_manipulator').perform(context).lower() in (
        'true', '1')
    rviz = LaunchConfiguration('rviz').perform(context).lower() in ('true', '1')
    delete_db = LaunchConfiguration('delete_db').perform(context).lower() in ('true', '1')
    port_name = LaunchConfiguration('port_name').perform(context)
    enable_lidar = LaunchConfiguration('enable_lidar').perform(context)
    lidar_type = LaunchConfiguration('lidar_type').perform(context)

    pkg_sim = get_package_share_directory('pro450_sim')
    actions = []

    # 1) Mobile base (+ optional full manipulator model for RViz)
    if start_bringup:
        if show_manipulator:
            nodes, msg = _mobile_manipulator_nodes(port_name)
            if nodes is None:
                actions.append(LogInfo(msg=msg))
                bringup = _agv_bringup_include(port_name, enable_lidar, lidar_type)
                if bringup is not None:
                    actions.append(bringup)
                    actions.append(LogInfo(msg='[slam] fallback: agv_pro_bringup (chassis only)'))
            else:
                actions.extend(nodes)
                actions.append(LogInfo(msg=msg))
        else:
            bringup = _agv_bringup_include(port_name, enable_lidar, lidar_type)
            if bringup is not None:
                actions.append(bringup)
                actions.append(LogInfo(
                    msg='[slam] starting agv_pro_bringup (chassis only; '
                        'show_manipulator:=true to see Pro450 arm)'))
            else:
                actions.append(LogInfo(
                    msg='[slam] agv_pro_bringup not found — source install/setup.bash '
                        'after: colcon build --packages-up-to agv_pro_bringup'))

    # 2) Orbbec 3D camera
    if start_camera:
        cam = _orbbec_gemini2_include()
        if cam is not None:
            actions.append(cam)
            actions.append(LogInfo(msg='[slam] starting orbbec_camera gemini2'))
        else:
            actions.append(LogInfo(
                msg='[slam] orbbec_camera not found — build orbbec_camera or '
                    'ros2 launch orbbec_camera gemini2.launch.py'))

    # Auto-pick backend
    if backend == 'auto':
        backend = 'rtabmap' if _pkg_share('rtabmap_slam') else 'depth_slam'

    if backend == 'rtabmap':
        if not _pkg_share('rtabmap_slam'):
            actions.append(LogInfo(
                msg='[slam] rtabmap_slam missing. Install: sudo apt install ros-humble-rtabmap-ros'))
            return actions

        parameters = {
            'frame_id': 'base_footprint',
            'odom_frame_id': 'odom',
            'use_sim_time': use_sim,
            'subscribe_depth': True,
            'subscribe_rgb': True,
            'approx_sync': True,
            'queue_size': 10,
            'Reg/Force3DoF': 'true',
            'Grid/FromDepth': 'true',
            'Grid/3D': 'false',
            'Grid/RangeMax': '3.5',
            'Grid/RayTracing': 'true',
            'Grid/MaxGroundHeight': '0.05',
            'Grid/MaxObstacleHeight': '0.6',
            'Grid/NormalsSegmentation': 'false',
            'Vis/MinInliers': '12',
            'RGBD/NeighborLinkRefining': 'true',
            'RGBD/ProximityBySpace': 'true',
            'RGBD/OptimizeFromGraphEnd': 'false',
            'Optimizer/GravitySigma': '0',
            'Mem/STMSize': '30',
        }
        remappings = [
            ('rgb/image', '/camera/color/image_raw'),
            ('rgb/camera_info', '/camera/color/camera_info'),
            ('depth/image', '/camera/depth/image_raw'),
            ('depth/camera_info', '/camera/depth/camera_info'),
        ]
        args = ['-d'] if delete_db else []
        actions.append(Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[parameters], remappings=remappings, arguments=args))
        actions.append(LogInfo(msg='[slam] backend=rtabmap (RGB-D from Orbbec)'))

    else:
        dil_share = _pkg_share('depthimage_to_laserscan')
        dil_params = {}
        if dil_share:
            cfg = os.path.join(dil_share, 'cfg', 'param.yaml')
            if os.path.isfile(cfg):
                dil_params = [cfg]
        actions.append(Node(
            package='depthimage_to_laserscan',
            executable='depthimage_to_laserscan_node',
            name='depthimage_to_laserscan',
            output='screen',
            parameters=dil_params if dil_params else [{
                'scan_time': 0.033,
                'range_min': 0.2,
                'range_max': 4.0,
                'scan_height': 10,
                'output_frame': 'camera_depth_frame',
            }],
            remappings=[
                ('depth', '/camera/depth/image_raw'),
                ('depth_camera_info', '/camera/depth/camera_info'),
                ('scan', '/scan'),
            ],
        ))
        slam_share = get_package_share_directory('slam_toolbox')
        slam_params = os.path.join(pkg_sim, 'config', 'mapper_params_online_async.yaml')
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(slam_share, 'launch', 'online_async_launch.py')),
            launch_arguments={
                'use_sim_time': 'true' if use_sim else 'false',
                'slam_params_file': slam_params,
            }.items(),
        ))
        actions.append(LogInfo(
            msg='[slam] backend=depth_slam (Orbbec depth → laserscan → slam_toolbox)'))

    if rviz:
        # Prefer our config so RobotModel (chassis+arm) is visible while mapping.
        cfg = os.path.join(pkg_sim, 'rviz', 'pro450_myagv_slam.rviz')
        actions.append(Node(
            package='rviz2', executable='rviz2', name='rviz2', output='screen',
            arguments=['-d', cfg],
            parameters=[{'use_sim_time': use_sim}],
        ))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('backend', default_value='auto',
                              description='auto|rtabmap|depth_slam'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'start_bringup', default_value='true',
            description='Launch base driver (and manipulator model if show_manipulator)'),
        DeclareLaunchArgument(
            'show_manipulator', default_value='true',
            description='Publish full myAGV+Pro450 URDF so RViz shows the arm '
                        '(arm held at home; teleop still only drives the base)'),
        DeclareLaunchArgument(
            'start_camera', default_value='true',
            description='Launch orbbec_camera gemini2'),
        DeclareLaunchArgument(
            'enable_lidar', default_value='false',
            description='Only used when show_manipulator:=false (full agv_pro_bringup)'),
        DeclareLaunchArgument('lidar_type', default_value='n10p'),
        DeclareLaunchArgument(
            'port_name', default_value='/dev/agvpro_controller'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('delete_db', default_value='true',
                              description='(rtabmap) wipe ~/.ros/rtabmap.db'),
        OpaqueFunction(function=_configure),
    ])
