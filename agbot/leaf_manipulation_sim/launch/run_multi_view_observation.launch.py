import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
import yaml


def load_yaml(package_name, relative_path):
    package_path = get_package_share_directory(package_name)
    with open(
        os.path.join(package_path, relative_path),
        encoding='utf-8',
    ) as stream:
        return yaml.safe_load(stream)


def generate_launch_description():
    pkg_sim = get_package_share_directory('leaf_manipulation_sim')
    moveit_pkg = get_package_share_directory('tm_moveit_config_tm5-900')
    urdf_path = os.path.join(pkg_sim, 'urdf', 'leaf_arm.sim.urdf.xacro')
    robot_description = {'robot_description': Command(['xacro ', urdf_path])}
    with open(
        os.path.join(moveit_pkg, 'config', 'tm5-900.srdf'),
        encoding='utf-8',
    ) as stream:
        semantic = stream.read().replace(
            '<virtual_joint name="virtual_joint" type="fixed" '
            'parent_frame="world" child_link="base" />',
            '',
        )

    maximum_views = LaunchConfiguration('maximum_views')
    settle_seconds = LaunchConfiguration('settle_seconds')
    return LaunchDescription([
        DeclareLaunchArgument('maximum_views', default_value='5'),
        DeclareLaunchArgument('settle_seconds', default_value='0.45'),
        Node(
            package='leaf_manipulation_sim',
            executable='multi_view_observation_demo',
            output='screen',
            parameters=[
                robot_description,
                {'robot_description_semantic': semantic},
                {
                    'robot_description_kinematics': load_yaml(
                        'tm_moveit_config_tm5-900',
                        'config/kinematics.yaml',
                    )
                },
                {
                    'maximum_views': maximum_views,
                    'settle_seconds': settle_seconds,
                    'use_sim_time': True,
                },
            ],
        ),
    ])
