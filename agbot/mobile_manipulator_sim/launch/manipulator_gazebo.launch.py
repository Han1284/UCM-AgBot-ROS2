import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_sim = get_package_share_directory('mobile_manipulator_sim')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('use_arm_control', default_value='false'),
        DeclareLaunchArgument('odom_topic', default_value='odom'),
        DeclareLaunchArgument('publish_odom_tf', default_value='true'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_sim, 'launch', 'spawn_mobile_manipulator.launch.py')),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'gui': LaunchConfiguration('gui'),
                'rviz': LaunchConfiguration('rviz'),
                'use_arm_control': LaunchConfiguration('use_arm_control'),
                'odom_topic': LaunchConfiguration('odom_topic'),
                'publish_odom_tf': LaunchConfiguration('publish_odom_tf'),
            }.items(),
        ),
    ])
