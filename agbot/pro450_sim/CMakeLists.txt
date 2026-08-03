cmake_minimum_required(VERSION 3.8)
project(pro450_sim)

find_package(ament_cmake REQUIRED)
install(DIRECTORY config launch models rviz worlds
  DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/generate_atrium_corridor_sdf.py
  scripts/generate_atrium_nav_map.py
  DESTINATION share/${PROJECT_NAME}/scripts
)
install(PROGRAMS
  scripts/pro450_static_joint_states.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME pro450_static_joint_states
)
install(PROGRAMS
  scripts/pro450_robot_description_topic.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME pro450_robot_description_topic
)
install(PROGRAMS
  scripts/pro450_mobile_tf_check.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME pro450_mobile_tf_check
)
install(PROGRAMS
  scripts/pro450_myagv_planar_base.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME pro450_myagv_planar_base
)
install(PROGRAMS
  scripts/pro450_myagv_wheel_joint_states.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME pro450_myagv_wheel_joint_states
)
install(PROGRAMS
  scripts/pro450_myagv_motion_demo.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME pro450_myagv_motion_demo
)
install(PROGRAMS scripts/pro450_plant_marker_publisher.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME pro450_plant_marker_publisher
)
install(PROGRAMS scripts/atrium_rviz_markers.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME atrium_rviz_markers
)
install(PROGRAMS scripts/pro450_myagv_gz_pose_sync.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME pro450_myagv_gz_pose_sync
)
install(PROGRAMS scripts/pro450_myagv_slam_mapping.sh
  DESTINATION lib/${PROJECT_NAME}
  RENAME pro450_myagv_slam_mapping
)
install(PROGRAMS scripts/pro450_myagv_atrium_slam.sh
  DESTINATION lib/${PROJECT_NAME}
  RENAME pro450_myagv_atrium_slam
)
install(PROGRAMS scripts/pro450_optical_cloud_to_scan.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME pro450_optical_cloud_to_scan
)
install(PROGRAMS scripts/pro450_myagv_atrium_patrol.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME pro450_myagv_atrium_patrol
)
install(PROGRAMS scripts/pro450_myagv_leaf_inspection.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME pro450_myagv_leaf_inspection
)
install(PROGRAMS scripts/pro450_myagv_leaf_inspection.sh
  DESTINATION lib/${PROJECT_NAME}
  RENAME pro450_myagv_leaf_inspection_bringup
)
install(PROGRAMS scripts/pro450_myagv_leaf_environment.sh
  DESTINATION lib/${PROJECT_NAME}
  RENAME pro450_myagv_leaf_environment
)
# Keep shell wrapper under a distinct name so the Python patrol node owns the short name.
install(PROGRAMS scripts/pro450_myagv_atrium_patrol.sh
  DESTINATION lib/${PROJECT_NAME}
  RENAME pro450_myagv_atrium_patrol_bringup
)
install(PROGRAMS scripts/pro450_leaf_pipeline.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME pro450_leaf_pipeline
)
ament_package()
