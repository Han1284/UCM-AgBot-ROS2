import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue


def generate_launch_description():
    pkg_sim = get_package_share_directory('panda_sim')
    pkg_prefix = get_package_prefix('panda_sim')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')
    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')

    urdf_path = os.path.join(pkg_sim, 'urdf', 'fixed_panda.urdf.xacro')
    sanitize_script = os.path.join(pkg_prefix, 'lib', 'panda_sim', 'sanitize_urdf_for_gazebo')
    urdf_out = '/tmp/fixed_panda_gazebo.urdf'
    rviz_config = os.path.join(pkg_sim, 'rviz', 'panda_sim.rviz')

    robot_description = ParameterValue(
        Command(['xacro ', urdf_path]),
        value_type=str,
    )

    spawn_panda = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-entity', 'fixed_panda',
            '-file', urdf_out,
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.0',
        ],
    )

    gzclient_after_spawn = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_panda,
            on_exit=[
                TimerAction(
                    period=0.5,
                    actions=[
                        ExecuteProcess(
                            cmd=['gzclient', '--verbose'],
                            output='screen',
                            additional_env={'LIBGL_ALWAYS_SOFTWARE': '1'},
                            condition=IfCondition(gui),
                        ),
                    ],
                ),
            ],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(pkg_sim, 'worlds', 'panda_empty.world'),
        ),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='false'),

        ExecuteProcess(
            cmd=[
                'bash', '-c',
                f'xacro "{urdf_path}" | python3 "{sanitize_script}" > "{urdf_out}"',
            ],
            output='screen',
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
            launch_arguments={
                'world': world,
                'use_sim_time': use_sim_time,
            }.items(),
        ),
        gzclient_after_spawn,

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_to_base_root_tf',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'base_root'],
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

        Node(
            package='panda_sim',
            executable='panda_static_joint_states',
            name='panda_static_joint_states',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),

        TimerAction(
            period=2.0,
            actions=[
                spawn_panda,
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
