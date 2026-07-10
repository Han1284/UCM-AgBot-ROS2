import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_sim = get_package_share_directory('leaf_manipulation_sim')
    params_file = os.path.join(pkg_sim, 'config', 'leaf_manipulation_params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')
    use_moveit = LaunchConfiguration('use_moveit')
    use_mock_poses = LaunchConfiguration('use_mock_poses')
    run_demo = LaunchConfiguration('run_demo')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('use_moveit', default_value='false'),
        DeclareLaunchArgument('use_mock_poses', default_value='true'),
        DeclareLaunchArgument('run_demo', default_value='true'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_sim, 'launch', 'leaf_sim_bringup.launch.py')),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'gui': gui,
                'rviz': rviz,
                'use_moveit': use_moveit,
            }.items(),
        ),

        Node(
            package='leaf_manipulation_sim',
            executable='mock_leaf_pose_publisher',
            name='mock_leaf_pose_publisher',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
            condition=IfCondition(use_mock_poses),
        ),

        Node(
            package='leaf_manipulation_sim',
            executable='leaf_pose_adapter',
            name='leaf_pose_adapter',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
        ),

        Node(
            package='leaf_manipulation_sim',
            executable='leaf_grasp_demo',
            name='leaf_grasp_demo',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
            condition=IfCondition(run_demo),
        ),
    ])
