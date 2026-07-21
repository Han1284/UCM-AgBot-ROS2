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
    kinematics_yaml = load_yaml('tm_moveit_config_tm5-900', 'config/kinematics.yaml')

    args = [
        'target_x', 'target_y', 'target_z',
        'target_roll', 'target_pitch', 'target_yaw',
        'target_offset_x', 'target_offset_y', 'target_offset_z',
        'finger_open', 'finger_closed',
        'velocity_scale', 'acceleration_scale', 'replay_duration_sec',
    ]
    defaults = {
        'target_x': 'nan',
        'target_y': 'nan',
        'target_z': 'nan',
        'target_roll': 'nan',
        'target_pitch': 'nan',
        'target_yaw': 'nan',
        'target_offset_x': '0.0',
        'target_offset_y': '-0.10',
        'target_offset_z': '-0.05',
        'finger_open': '0.10',
        'finger_closed': '0.65',
        'velocity_scale': '0.2',
        'acceleration_scale': '0.2',
        'replay_duration_sec': '3.0',
    }

    launch_actions = [
        DeclareLaunchArgument(name, default_value=defaults[name])
        for name in args
    ]

    launch_actions.append(
        Node(
            package='leaf_manipulation_sim',
            executable='move_to_pose_grasp_demo',
            output='screen',
            parameters=[
                robot_description,
                robot_description_semantic,
                {'robot_description_kinematics': kinematics_yaml},
                {'use_sim_time': True},
                *[{name: LaunchConfiguration(name)} for name in args],
            ],
        )
    )

    return LaunchDescription(launch_actions)
