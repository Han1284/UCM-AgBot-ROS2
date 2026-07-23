import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
import yaml


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    with open(absolute_file_path, 'r', encoding='utf-8') as file:
        return yaml.safe_load(file)


def generate_launch_description():
    pkg_sim = get_package_share_directory('leaf_manipulation_sim')
    radius = LaunchConfiguration('radius')
    repetitions = LaunchConfiguration('repetitions')
    samples_per_circle = LaunchConfiguration('samples_per_circle')
    velocity_scale = LaunchConfiguration('velocity_scale')
    acceleration_scale = LaunchConfiguration('acceleration_scale')
    finger_position = LaunchConfiguration('finger_position')

    urdf_path = os.path.join(pkg_sim, 'urdf', 'leaf_arm.sim.urdf.xacro')
    robot_description = {'robot_description': Command(['xacro ', urdf_path])}
    robot_description_semantic = {
        'robot_description_semantic': open(
            os.path.join(
                get_package_share_directory('tm_moveit_config_tm5-900'),
                'config',
                'tm5-900.srdf',
            ),
            encoding='utf-8',
        ).read().replace(
            '<virtual_joint name="virtual_joint" type="fixed" parent_frame="world" child_link="base" />',
            '',
        )
    }
    kinematics_yaml = load_yaml(
        'tm_moveit_config_tm5-900', 'config/kinematics.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('radius', default_value='0.10'),
        DeclareLaunchArgument('repetitions', default_value='3'),
        DeclareLaunchArgument('samples_per_circle', default_value='60'),
        DeclareLaunchArgument('velocity_scale', default_value='0.2'),
        DeclareLaunchArgument('acceleration_scale', default_value='0.2'),
        DeclareLaunchArgument('finger_position', default_value='0.10'),
        Node(
            package='leaf_manipulation_sim',
            executable='circle_motion_demo',
            output='screen',
            parameters=[
                robot_description,
                robot_description_semantic,
                {'robot_description_kinematics': kinematics_yaml},
                {'radius': radius},
                {'repetitions': repetitions},
                {'samples_per_circle': samples_per_circle},
                {'velocity_scale': velocity_scale},
                {'acceleration_scale': acceleration_scale},
                {'finger_position': finger_position},
                {'use_sim_time': True},
            ],
        ),
    ])
