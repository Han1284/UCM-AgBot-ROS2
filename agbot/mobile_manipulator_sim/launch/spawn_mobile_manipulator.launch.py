import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_sim = get_package_share_directory('mobile_manipulator_sim')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    yaw_pose = LaunchConfiguration('yaw_pose')
    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')
    use_arm_control = LaunchConfiguration('use_arm_control')
    odom_topic = LaunchConfiguration('odom_topic')
    publish_odom_tf = LaunchConfiguration('publish_odom_tf')

    urdf_path = os.path.join(pkg_sim, 'urdf', 'mobile_manipulator.sim.urdf.xacro')
    rviz_config = os.path.join(pkg_sim, 'rviz', 'mobile_manipulator_sim.rviz')
    controllers_file = os.path.join(pkg_sim, 'config', 'mobile_manipulator_controllers.yaml')

    robot_description = Command([
        'xacro ', urdf_path,
        ' odom_topic:=', odom_topic,
        ' publish_odom_tf:=', publish_odom_tf,
        ' use_arm_control:=', use_arm_control,
    ])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(
                get_package_share_directory('robot_simulator'), 'worlds', 'orchard.world'),
            description='Gazebo world file'),
        DeclareLaunchArgument('x_pose', default_value='0.0'),
        DeclareLaunchArgument('y_pose', default_value='0.0'),
        DeclareLaunchArgument('yaw_pose', default_value='1.5707'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('use_arm_control', default_value='false'),
        DeclareLaunchArgument('odom_topic', default_value='odom'),
        DeclareLaunchArgument('publish_odom_tf', default_value='true'),

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

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': robot_description,
                'source_list': ['joint_states'],
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
                        '-entity', 'mobile_manipulator',
                        '-topic', 'robot_description',
                        '-x', x_pose,
                        '-y', y_pose,
                        '-z', '0.0',
                        '-Y', yaw_pose,
                    ],
                ),
            ],
        ),

        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='controller_manager',
                    executable='spawner',
                    output='screen',
                    arguments=['joint_state_controller'],
                    parameters=[{'use_sim_time': use_sim_time}],
                    condition=IfCondition(use_arm_control),
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
