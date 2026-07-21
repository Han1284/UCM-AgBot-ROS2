import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue


def generate_launch_description():
    pkg_sim = get_package_share_directory('leaf_manipulation_sim')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_tm = get_package_share_directory('tm_description')
    pkg_rg = get_package_share_directory('onrobot_rg_description')
    gazebo_share = '/usr/share/gazebo-11'
    gazebo_media = '/usr/share/gazebo-11/media'
    gazebo_models = '/usr/share/gazebo-11/models'
    gazebo_plugins = '/usr/lib/x86_64-linux-gnu/gazebo-11/plugins'
    ogre_resource_path = '/usr/lib/x86_64-linux-gnu/OGRE-1.9.0'

    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')
    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')
    software_gzclient = LaunchConfiguration('software_gzclient')
    publish_static_joint_states = LaunchConfiguration('publish_static_joint_states')

    urdf_path = os.path.join(pkg_sim, 'urdf', 'fixed_tm5_rg2.urdf.xacro')
    sanitize_script = os.path.join(pkg_sim, 'urdf', 'sanitize_urdf_for_gazebo.py')
    urdf_out = '/tmp/fixed_tm5_rg2_gazebo.urdf'
    rviz_config = os.path.join(pkg_sim, 'rviz', 'leaf_manipulation.rviz')

    robot_description = ParameterValue(
        Command(['xacro ', urdf_path]),
        value_type=str,
    )

    declared_args = [
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(pkg_sim, 'worlds', 'fixed_arm_empty.world'),
            description='Gazebo world file'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('software_gzclient', default_value='false'),
        DeclareLaunchArgument('publish_static_joint_states', default_value='true'),
    ]

    gazebo_resource_path = SetEnvironmentVariable(
        name='GAZEBO_RESOURCE_PATH',
        value=':'.join([
            gazebo_share,
            gazebo_media,
            pkg_tm,
            pkg_rg,
            pkg_sim,
            os.environ.get('GAZEBO_RESOURCE_PATH', ''),
        ]),
    )

    gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=':'.join([
            os.path.join(pkg_sim, 'models'),
            os.path.dirname(pkg_tm),
            os.path.dirname(pkg_rg),
            gazebo_models,
            os.environ.get('GAZEBO_MODEL_PATH', ''),
        ]),
    )

    gazebo_model_database = SetEnvironmentVariable(
        name='GAZEBO_MODEL_DATABASE_URI',
        value='',
    )

    gazebo_plugin_path = SetEnvironmentVariable(
        name='GAZEBO_PLUGIN_PATH',
        value=':'.join([
            gazebo_plugins,
            os.environ.get('GAZEBO_PLUGIN_PATH', ''),
        ]),
    )

    ogre_resources = SetEnvironmentVariable(
        name='OGRE_RESOURCE_PATH',
        value=':'.join([
            ogre_resource_path,
            os.environ.get('OGRE_RESOURCE_PATH', ''),
        ]),
    )

    # VMware's SVGA3D OpenGL 3 path can stall Gazebo depth-camera FBOs.
    vmware_gl_compatibility = SetEnvironmentVariable(
        name='SVGA_VGPU10',
        value='0',
    )

    qt_shared_memory = SetEnvironmentVariable(
        name='QT_X11_NO_MITSHM',
        value='1',
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

    gzclient = ExecuteProcess(
        cmd=[
            'bash',
            '-lc',
            PythonExpression([
                '"LIBGL_ALWAYS_SOFTWARE=true gzclient" if "true" == "',
                software_gzclient,
                '" else "gzclient"',
            ]),
        ],
        output='screen',
        condition=IfCondition(gui),
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

    return LaunchDescription([
        *declared_args,
        gazebo_resource_path,
        gazebo_model_path,
        gazebo_model_database,
        gazebo_plugin_path,
        ogre_resources,
        vmware_gl_compatibility,
        qt_shared_memory,
        generate_urdf,
        gzserver,
        gzclient,

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_to_base_root_tf',
            output='screen',
            arguments=['--frame-id', 'world', '--child-frame-id', 'base_root'],
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
                    parameters=[
                        {'use_sim_time': use_sim_time},
                        {'joint_names': [
                            'joint_1', 'joint_2', 'joint_3',
                            'joint_4', 'joint_5', 'joint_6', 'finger_joint',
                        ]},
                        {'joint_positions': [
                            0.0, 0.0, 1.5708, 0.0, 1.5708, 0.0, 0.78,
                        ]},
                    ],
                    condition=IfCondition(publish_static_joint_states),
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
