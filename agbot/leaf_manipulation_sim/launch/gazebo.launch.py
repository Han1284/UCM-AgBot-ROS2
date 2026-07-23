import os

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
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    pkg_sim = get_package_share_directory('leaf_manipulation_sim')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_tm = get_package_share_directory('tm_description')
    pkg_rg = get_package_share_directory('onrobot_rg_description')
    pkg_realsense = get_package_share_directory('realsense2_description')

    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')
    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')
    spawn_robot = LaunchConfiguration('spawn_robot')

    urdf_path = os.path.join(pkg_sim, 'urdf', 'leaf_arm.sim.urdf.xacro')
    rviz_config = os.path.join(pkg_sim, 'rviz', 'leaf_manipulation_sim.rviz')
    controllers_file = os.path.join(pkg_sim, 'config', 'arm_controllers.yaml')
    gui_config = os.path.join(pkg_sim, 'config', 'gazebo_gui.config')
    robot_description = Command(['xacro ', urdf_path])

    # Keep the official Collada mesh in robot_description for RViz.  Fortress's
    # Ogre2 path renders that legacy DAE incorrectly, so only the entity sent to
    # Gazebo uses an Assimp-triangulated OBJ generated from the same geometry.
    gazebo_robot_description = xacro.process_file(urdf_path).toxml()
    d435_dae = 'package://realsense2_description/meshes/d435.dae'
    d435_obj = 'package://leaf_manipulation_sim/meshes/d435_fortress.obj'
    if gazebo_robot_description.count(d435_dae) != 1:
        raise RuntimeError('Expected exactly one D435 visual mesh in robot URDF')
    gazebo_robot_description = gazebo_robot_description.replace(
        d435_dae, d435_obj)

    resource_path = os.pathsep.join([
        os.path.join(pkg_sim, 'models'),
        os.path.dirname(pkg_sim),
        os.path.dirname(pkg_tm),
        os.path.dirname(pkg_rg),
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
        launch_arguments={'gz_args': ['-r -s ', world], 'gz_version': '6'}.items(),
        condition=UnlessCondition(gui),
    )

    sensor_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='fortress_sensor_bridge',
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
            ('/camera_gz/points', '/camera/depth/color/points'),
        ],
    )

    depth_info_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='fortress_depth_info_bridge',
        output='screen',
        arguments=[
            '/camera_gz/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        ],
        remappings=[
            ('/camera_gz/camera_info', '/camera/depth/camera_info'),
        ],
    )

    return LaunchDescription([
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resource_path),
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', resource_path),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(pkg_sim, 'worlds', 'leaf_bench.world'),
            description='Gazebo Fortress world for the leaf manipulation bench'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('spawn_robot', default_value='true'),
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
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': robot_description,
            }],
        ),
        TimerAction(
            period=3.0,
            actions=[Node(
                package='ros_gz_sim',
                executable='create',
                output='screen',
                arguments=[
                    '-name', 'leaf_arm',
                    '-param', 'robot_description',
                    '-x', '0.0', '-y', '0.0', '-z', '0.0',
                ],
                parameters=[{
                    'robot_description': gazebo_robot_description,
                }],
                condition=IfCondition(spawn_robot),
            )],
        ),
        TimerAction(
            period=7.0,
            actions=[Node(
                package='controller_manager',
                executable='spawner',
                output='screen',
                arguments=[
                    'joint_state_broadcaster',
                    'arm_position_controller',
                    'gripper_position_controller',
                    '--param-file', controllers_file,
                    '--controller-manager-timeout', '30',
                ],
            )],
        ),
        TimerAction(
            period=7.5,
            actions=[Node(
                package='leaf_manipulation_sim',
                executable='initial_controller_commands',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}],
            )],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(rviz),
        ),
    ])
