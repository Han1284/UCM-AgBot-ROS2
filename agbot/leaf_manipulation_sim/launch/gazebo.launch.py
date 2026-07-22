import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_sim = get_package_share_directory('leaf_manipulation_sim')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
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
    models_path = os.path.join(pkg_sim, 'models')

    robot_description = Command(['xacro ', urdf_path])

    gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=[
            models_path,
            ':', os.path.dirname(pkg_tm),
            ':', os.path.dirname(pkg_rg),
            ':', os.path.dirname(pkg_realsense),
            ':/usr/share/gazebo-11/models:',
            os.environ.get('GAZEBO_MODEL_PATH', ''),
        ],
    )
    gazebo_model_database = SetEnvironmentVariable(
        name='GAZEBO_MODEL_DATABASE_URI',
        value='',
    )

    return LaunchDescription([
        gazebo_model_path,
        gazebo_model_database,
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(pkg_sim, 'worlds', 'leaf_bench.world'),
            description='Gazebo world file for leaf manipulation bench'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('spawn_robot', default_value='true'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
            launch_arguments={'world': world}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')),
            condition=IfCondition(gui),
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
            actions=[
                Node(
                    package='gazebo_ros',
                    executable='spawn_entity.py',
                    output='screen',
                    arguments=[
                        '-entity', 'leaf_arm',
                        '-topic', 'robot_description',
                        '-x', '0.0',
                        '-y', '0.0',
                        '-z', '0.0',
                    ],
                    condition=IfCondition(spawn_robot),
                ),
            ],
        ),

        TimerAction(
            period=6.0,
            actions=[
                Node(
                    package='controller_manager',
                    executable='spawner',
                    output='screen',
                    arguments=[
                        'joint_state_broadcaster',
                        'arm_position_controller',
                        'gripper_position_controller',
                        '--param-file', controllers_file,
                    ],
                ),
            ],
        ),

        TimerAction(
            period=6.5,
            actions=[
                Node(
                    package='leaf_manipulation_sim',
                    executable='initial_controller_commands',
                    output='screen',
                    parameters=[{'use_sim_time': use_sim_time}],
                ),
            ],
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
