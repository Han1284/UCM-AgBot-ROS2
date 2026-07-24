import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_sim = get_package_share_directory('leaf_manipulation_sim')

    replay_duration = LaunchConfiguration('replay_duration_sec')
    settle_duration = LaunchConfiguration('target_settle_sec')
    grasp_hold_duration = LaunchConfiguration('grasp_hold_sec')

    # Leaf 1, segment 5 is the outer leaf-tip proxy.  This pose aligns the RG2
    # closing axis with the leaf normal and keeps link_6 and the D435 clear of
    # the plant.  Coordinates are expressed in the robot base frame.
    leaf_target = {
        'target_x': '0.7090',
        'target_y': '-0.2410',
        # The C++ demo accepts base-frame coordinates; base is 0.05 m above
        # world, while the calibrated leaf center is world z=0.529 m.
        'target_z': '0.4790',
        'target_roll': '1.570796',
        'target_pitch': '-1.543496',
        'target_yaw': '1.119393',
    }

    return LaunchDescription([
        DeclareLaunchArgument('replay_duration_sec', default_value='6.0'),
        DeclareLaunchArgument('target_settle_sec', default_value='8.0'),
        DeclareLaunchArgument('grasp_hold_sec', default_value='2.0'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    pkg_sim, 'launch', 'run_pose_grasp_demo.launch.py')),
            launch_arguments={
                **leaf_target,
                'finger_open': '0.10',
                'finger_closed': '0.680',
                'replay_duration_sec': replay_duration,
                'target_settle_sec': settle_duration,
                'grasp_hold_sec': grasp_hold_duration,
                'hold_after_run': 'false',
            }.items(),
        ),
    ])
