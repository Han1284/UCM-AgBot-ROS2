import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue


def generate_launch_description():
    pkg_sim = get_package_share_directory('leaf_manipulation_sim')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_tm = get_package_share_directory('tm_description')
    pkg_rg = get_package_share_directory('onrobot_rg_description')

    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')
    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')

    urdf_path = os.path.join(pkg_sim, 'urdf', 'fixed_tm5_rg2.urdf.xacro')
    sanitize_script = os.path.join(pkg_sim, 'urdf', 'sanitize_urdf_for_gazebo.py')
    urdf_out = '/tmp/fixed_tm5_rg2_gazebo.urdf'
    rviz_config = os.path.join(pkg_sim, 'rviz', 'leaf_manipulation_sim.rviz')

    robot_description = ParameterValue(
        Command([
            'bash -c \'xacro "', urdf_path, '" | python3 "', sanitize_script, '"\'',
        ]),
        value_type=str,
    )

    declared_args = [
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(pkg_sim, 'worlds', 'fixed_arm_empty.world'),
            description='Gazebo world file'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='false'),
    ]

    gazebo_resource_path = SetEnvironmentVariable(
        name='GAZEBO_RESOURCE_PATH',
        value=':'.join([
            pkg_tm,
            pkg_rg,
            os.environ.get('GAZEBO_RESOURCE_PATH', ''),
        ]),
    )

    generate_urdf = ExecuteProcess(
        cmd=[
            'bash', '-c',
            f'xacro "{urdf_path}" | python3 "{sanitize_script}" > "{urdf_out}"',
        ],
        output='screen',
    )

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
        launch_arguments={
            'world': world,
            'use_sim_time': use_sim_time,
        }.items(),
    )

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-entity', 'fixed_tm5_rg2',
            '-file', urdf_out,
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.0',
            '-timeout', '60',
        ],
    )

    gzclient_after_spawn = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=[
                ExecuteProcess(
                    cmd=[
                        'gnome-terminal',
                        '--',
                        'bash',
                        '-lc',
                        'LIBGL_ALWAYS_SOFTWARE=1 gzclient',
                    ],
                    output='screen',
                    condition=IfCondition(gui),
                ),
            ],
        ),
    )

    return LaunchDescription([
        *declared_args,
        gazebo_resource_path,
        generate_urdf,
        gzserver,
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

        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='leaf_manipulation_sim',
                    executable='static_joint_states',
                    name='static_joint_states',
                    output='screen',
                    parameters=[{'use_sim_time': use_sim_time}],
                ),
            ],
        ),

        TimerAction(
            period=4.0,
            actions=[
                spawn_robot,
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
