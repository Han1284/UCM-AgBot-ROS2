import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
import yaml


def _load_yaml(package, relative_path):
    with open(
        os.path.join(get_package_share_directory(package), relative_path),
        encoding='utf-8',
    ) as stream:
        return yaml.safe_load(stream)


def generate_launch_description():
    description_share = get_package_share_directory('pro450_description')
    moveit_share = get_package_share_directory('pro450_moveit_config')
    urdf = os.path.join(
        description_share, 'urdf', 'pro450_f100.urdf.xacro')
    with open(
        os.path.join(moveit_share, 'config', 'pro450_f100.srdf'),
        encoding='utf-8',
    ) as stream:
        semantic = stream.read()
    return LaunchDescription([
        DeclareLaunchArgument('maximum_views', default_value='5'),
        DeclareLaunchArgument('settle_seconds', default_value='0.45'),
        Node(
            package='leaf_manipulation_sim',
            executable='multi_view_observation_demo',
            output='screen',
            parameters=[
                {'robot_description': Command([
                    'xacro ', urdf,
                    ' use_gz_control:=true collision_mesh_scale:=0.001'])},
                {'robot_description_semantic': semantic},
                {'robot_description_kinematics': _load_yaml(
                    'pro450_moveit_config', 'config/kinematics.yaml')},
                {
                    'arm_group': 'pro450_arm',
                    'base_frame': 'base_root',
                    'end_effector_link': 'gripper_base',
                    'arm_joint_names': [
                        'joint1', 'joint2', 'joint3',
                        'joint4', 'joint5', 'joint6',
                    ],
                    # Execute observation trajectories through the same
                    # FollowJointTrajectory path used by MTC.  This avoids
                    # two controllers claiming the arm joints.
                    'execute_via_move_group': True,
                    'initial_observation_joint_positions':
                        [0.0, 0.4, -1.2, 0.0, 0.0, 0.4],
                    'maximum_views': LaunchConfiguration('maximum_views'),
                    'settle_seconds': LaunchConfiguration('settle_seconds'),
                    'use_sim_time': True,
                },
            ],
        ),
    ])
