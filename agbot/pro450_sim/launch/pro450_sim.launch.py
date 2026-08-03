"""Gazebo Fortress visual simulation for the independent myCobot Pro 450.

This deliberately reuses the established leaf-bench world and RViz layout.
The existing TM5/RG2 simulation remains untouched.
"""

import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
import xacro


def gazebo_description(urdf_path, controlled=False):
    """Supply the inertias Fortress requires without changing the RViz URDF."""
    robot = ET.fromstring(xacro.process_file(
        urdf_path,
        mappings={'use_gz_control': 'true' if controlled else 'false'},
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

    if not controlled:
        gazebo = ET.SubElement(robot, 'gazebo')
        ET.SubElement(gazebo, 'static').text = 'true'
    return ET.tostring(robot, encoding='unicode')


def generate_launch_description():
    pkg_sim = get_package_share_directory('pro450_sim')
    pkg_description = get_package_share_directory('pro450_description')
    pkg_moveit = get_package_share_directory('pro450_moveit_config')
    pkg_leaf = get_package_share_directory('leaf_manipulation_sim')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_realsense = get_package_share_directory('realsense2_description')

    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')
    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')
    spawn_robot = LaunchConfiguration('spawn_robot')
    control = LaunchConfiguration('control')
    transport_partition = f'pro450_sim_{os.getpid()}'

    urdf_path = os.path.join(pkg_description, 'urdf', 'pro450_f100.urdf.xacro')
    srdf_path = os.path.join(pkg_moveit, 'config', 'pro450_f100.srdf')
    rviz_config = os.path.join(pkg_sim, 'rviz', 'pro450_sim.rviz')
    gui_config = os.path.join(pkg_leaf, 'config', 'gazebo_gui.config')
    robot_description = Command([
        'xacro ', urdf_path,
        ' use_gz_control:=', control,
        ' collision_mesh_scale:=0.001'])
    with open(srdf_path, encoding='utf-8') as srdf_file:
        robot_description_semantic = srdf_file.read()
    static_gazebo_robot_description = gazebo_description(
        urdf_path, controlled=False)
    controlled_gazebo_robot_description = gazebo_description(
        urdf_path, controlled=True)

    # Fortress resolves package:// meshes through this resource path.  Keep the
    # original leaf world and all of its models available without copying them.
    resource_path = os.pathsep.join([
        os.path.join(pkg_sim, 'models'),
        os.path.join(pkg_leaf, 'models'),
        os.path.dirname(pkg_leaf),
        os.path.dirname(pkg_description),
        os.path.dirname(pkg_realsense),
        os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
    ])

    gz_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': ['-r ', world, ' --gui-config ', gui_config],
            'gz_version': '6',
        }.items(),
        condition=IfCondition(gui),
    )
    gz_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': ['-r -s ', world],
            'gz_version': '6',
        }.items(),
        condition=UnlessCondition(gui),
    )

    sensor_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='pro450_fortress_sensor_bridge',
        output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/camera_gz/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera_gz/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/camera_gz/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera_gz/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/plant/leaf_1_contacts_gz@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts',
            '/plant/leaf_2_contacts_gz@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts',
            '/plant/leaf_3_contacts_gz@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts',
            '/plant/leaf_4_contacts_gz@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts',
        ],
        remappings=[
            ('/camera_gz/image', '/camera/color/image_raw'),
            ('/camera_gz/camera_info', '/camera/color/camera_info'),
            ('/camera_gz/depth_image', '/camera/depth/image_raw'),
            ('/camera_gz/points', '/camera/depth/color/points_gz'),
        ],
    )

    depth_info_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='pro450_fortress_depth_info_bridge',
        output='screen',
        arguments=['/camera_gz/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'],
        remappings=[('/camera_gz/camera_info', '/camera/depth/camera_info')],
    )

    return LaunchDescription([
        SetEnvironmentVariable('IGN_PARTITION', transport_partition),
        SetEnvironmentVariable('GZ_PARTITION', transport_partition),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resource_path),
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', resource_path),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(pkg_sim, 'worlds', 'pro450_leaf_bench.world'),
            description='Pro450-scaled Gazebo Fortress leaf-bench environment'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('spawn_robot', default_value='true'),
        DeclareLaunchArgument(
            'control',
            default_value='true',
            description=(
                'Enable gz_ros2_control for real multi-view arm motion; '
                'set false only for static visual inspection')),
        gz_gui,
        gz_server,
        sensor_bridge,
        depth_info_bridge,
        Node(
            package='leaf_manipulation_sim',
            executable='contact_bridge_adapter',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        Node(
            package='leaf_manipulation_sim',
            executable='point_cloud_optical_adapter',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                # The Pro450 Fortress sensor uses zero local yaw.  Convert
                # Gazebo X-forward/Y-left/Z-up to ROS optical coordinates.
                'invert_optical_x': True,
                'invert_optical_y': True,
            }],
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': robot_description,
            }],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_to_base_root_tf',
            output='screen',
            arguments=[
                '--frame-id', 'world',
                '--child-frame-id', 'base_root',
            ],
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        Node(
            package='pro450_sim',
            executable='pro450_static_joint_states',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            condition=UnlessCondition(control),
        ),
        TimerAction(
            # Fortress must finish registering its world service before create
            # queries it.  GUI initialization can take several seconds.
            period=8.0,
            actions=[Node(
                package='ros_gz_sim',
                executable='create',
                output='screen',
                arguments=[
                    '-name', 'pro450_f100',
                    '-param', 'robot_description',
                    '-x', '0.0', '-y', '0.0', '-z', '0.0',
                ],
                parameters=[{
                    'robot_description': static_gazebo_robot_description}],
                condition=IfCondition(PythonExpression([
                    "'", spawn_robot, "' == 'true' and '",
                    control, "' == 'false'"])),
            )],
        ),
        TimerAction(
            period=8.0,
            actions=[Node(
                package='ros_gz_sim',
                executable='create',
                output='screen',
                arguments=[
                    '-name', 'pro450_f100',
                    '-param', 'robot_description',
                    '-x', '0.0', '-y', '0.0', '-z', '0.0',
                ],
                parameters=[{
                    'robot_description': controlled_gazebo_robot_description}],
                condition=IfCondition(PythonExpression([
                    "'", spawn_robot, "' == 'true' and '",
                    control, "' == 'true'"])),
            )],
        ),
        TimerAction(
            period=12.0,
            actions=[
                Node(
                    package='controller_manager',
                    executable='spawner',
                    output='screen',
                    arguments=[
                        'joint_state_broadcaster',
                        '--controller-manager', '/controller_manager',
                        '--controller-manager-timeout', '60.0',
                    ],
                    condition=IfCondition(control),
                ),
            ],
        ),
        # Start the command controllers only after the state broadcaster has
        # been loaded.  On a cold Fortress startup the controller manager can
        # appear several seconds after the entity is created.
        TimerAction(
            period=15.0,
            actions=[
                Node(
                    package='controller_manager',
                    executable='spawner',
                    output='screen',
                    arguments=[
                        'arm_trajectory_controller',
                        '--controller-manager', '/controller_manager',
                        '--controller-manager-timeout', '60.0',
                    ],
                    condition=IfCondition(control),
                ),
                Node(
                    package='controller_manager',
                    executable='spawner',
                    output='screen',
                    arguments=[
                        'gripper_trajectory_controller',
                        '--controller-manager', '/controller_manager',
                        '--controller-manager-timeout', '60.0',
                    ],
                    condition=IfCondition(control),
                ),
            ],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': robot_description,
                # The MotionPlanning display needs the Pro450 semantic groups.
                'robot_description_semantic': robot_description_semantic,
            }],
            condition=IfCondition(rviz),
        ),
        Node(
            package='pro450_sim',
            executable='pro450_plant_marker_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(rviz),
        ),
    ])
