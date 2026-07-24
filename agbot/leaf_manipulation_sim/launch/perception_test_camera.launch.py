import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_sim = get_package_share_directory('leaf_manipulation_sim')
    model_path = os.path.join(
        pkg_sim, 'models', 'perception_test_camera', 'model.sdf')
    label_model_path = os.path.join(
        pkg_sim, 'models', 'perception_label_plant', 'model.sdf')

    world = LaunchConfiguration('world')
    name = LaunchConfiguration('name')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    roll = LaunchConfiguration('roll')
    pitch = LaunchConfiguration('pitch')
    yaw = LaunchConfiguration('yaw')
    spawn_delay = LaunchConfiguration('spawn_delay')
    training_labels = LaunchConfiguration('training_labels')

    spawn_camera = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_perception_test_camera',
        output='screen',
        arguments=[
            '-world', world,
            '-file', model_path,
            '-name', name,
            '-allow_renaming', 'false',
            '-x', x,
            '-y', y,
            '-z', z,
            '-R', roll,
            '-P', pitch,
            '-Y', yaw,
        ],
    )

    spawn_label_plant = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_perception_label_plant',
        output='screen',
        condition=IfCondition(training_labels),
        arguments=[
            '-world', world,
            '-file', label_model_path,
            '-name', 'perception_label_plant',
            '-allow_renaming', 'false',
            '-x', '0.85',
            '-y', '0.0',
            '-z', '-10.0',
        ],
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='perception_test_camera_bridge',
        output='screen',
        arguments=[
            '/perception_test_camera_gz/image'
            '@sensor_msgs/msg/Image[gz.msgs.Image',
            '/perception_test_camera_gz/camera_info'
            '@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/perception_test_camera_gz/depth_image'
            '@sensor_msgs/msg/Image[gz.msgs.Image',
            '/perception_test_camera_gz/points'
            '@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
        ],
        remappings=[
            ('/perception_test_camera_gz/image',
             '/perception_test_camera/color/image_raw'),
            ('/perception_test_camera_gz/camera_info',
             '/perception_test_camera/color/camera_info'),
            ('/perception_test_camera_gz/depth_image',
             '/perception_test_camera/depth/image_raw'),
            ('/perception_test_camera_gz/points',
             '/perception_test_camera/depth/color/points'),
        ],
    )

    depth_info_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='perception_test_camera_depth_info_bridge',
        output='screen',
        arguments=[
            '/perception_test_camera_gz/camera_info'
            '@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        ],
        remappings=[
            ('/perception_test_camera_gz/camera_info',
             '/perception_test_camera/depth/camera_info'),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='leaf_bench',
            description='Name of the already running Gazebo world'),
        DeclareLaunchArgument(
            'name',
            default_value='perception_test_camera',
            description='Gazebo entity name'),
        DeclareLaunchArgument(
            'x', default_value='0.85',
            description='Camera world X position in metres'),
        DeclareLaunchArgument(
            'y', default_value='0.0',
            description='Camera world Y position in metres'),
        DeclareLaunchArgument(
            'z', default_value='1.35',
            description='Camera world Z position in metres'),
        DeclareLaunchArgument(
            'roll', default_value='0.0',
            description='Camera world roll in radians'),
        DeclareLaunchArgument(
            'pitch', default_value=str(math.pi / 2.0),
            description='Camera world pitch in radians; +pi/2 looks down'),
        DeclareLaunchArgument(
            'yaw', default_value='0.0',
            description='Camera world yaw in radians'),
        DeclareLaunchArgument(
            'spawn_delay', default_value='1.0',
            description='Seconds to wait for an already running Gazebo world'),
        DeclareLaunchArgument(
            'training_labels', default_value='false',
            description='Spawn the hidden leaf-only model used for labels'),
        TimerAction(
            period=spawn_delay,
            actions=[spawn_camera, spawn_label_plant],
        ),
        bridge,
        depth_info_bridge,
    ])
