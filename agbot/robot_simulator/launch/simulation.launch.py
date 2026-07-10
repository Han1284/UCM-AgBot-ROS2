import os
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression, Command
from launch_ros.actions import Node


def _package_available(package_name):
    try:
        get_package_share_directory(package_name)
    except PackageNotFoundError:
        return False
    return True


def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_robot_sim = get_package_share_directory('robot_simulator')
    has_robot_localization = _package_available('robot_localization')

    urdf_path = os.path.join(pkg_robot_sim, 'urdf_sim', 'robot_description_gazebo.urdf.xacro')
    world_path = os.path.join(pkg_robot_sim, 'worlds', 'orchard.world')
    ekf_config_path = os.path.join(pkg_robot_sim, 'config', 'ekf.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    Y_pose = LaunchConfiguration('Y_pose', default='1.5707')
    use_robot_localization = LaunchConfiguration(
        'use_robot_localization', default='true' if has_robot_localization else 'false'
    )
    odom_topic = LaunchConfiguration(
        'odom_topic', default='wheel/odometry' if has_robot_localization else 'odom'
    )
    publish_odom_tf = LaunchConfiguration(
        'publish_odom_tf', default='false' if has_robot_localization else 'true'
    )

    robot_desc = Command([
        'xacro ',
        urdf_path,
        ' odom_topic:=',
        odom_topic,
        ' publish_odom_tf:=',
        publish_odom_tf,
    ])

    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world_path}.items(),
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        ),
        condition=IfCondition(PythonExpression(['True', ' and not ', 'False']))
    )

    gazebo_spawner_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'my_robot',
            '-x', x_pose,
            '-y', y_pose,
            '-z', '0.0',
            '-Y', Y_pose,
            '-topic', 'robot_description'
        ],
        output='screen',
    )

    robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace='',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc}],
        remappings=remappings,
        arguments=[robot_desc]
    )

    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path, {'use_sim_time': use_sim_time}],
        remappings=[('/set_pose', '/initialpose')],
        condition=IfCondition(use_robot_localization),
    )

    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument(
        'use_robot_localization',
        default_value='true' if has_robot_localization else 'false',
        description='Use robot_localization EKF if the package is available.'
    ))
    ld.add_action(DeclareLaunchArgument(
        'odom_topic',
        default_value='wheel/odometry' if has_robot_localization else 'odom',
        description='Odometry topic published by the Gazebo diff-drive plugin.'
    ))
    ld.add_action(DeclareLaunchArgument(
        'publish_odom_tf',
        default_value='false' if has_robot_localization else 'true',
        description='Whether Gazebo publishes the odom to base_link transform.'
    ))
    ld.add_action(LogInfo(
        condition=UnlessCondition(use_robot_localization),
        msg='robot_localization not available; using Gazebo odometry directly.'
    ))
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(gazebo_spawner_cmd)
    ld.add_action(robot_localization_node)

    return ld
