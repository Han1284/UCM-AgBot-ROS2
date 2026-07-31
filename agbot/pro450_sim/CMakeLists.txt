cmake_minimum_required(VERSION 3.8)
project(pro450_sim)

find_package(ament_cmake REQUIRED)
install(DIRECTORY config launch models rviz worlds
  DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/pro450_static_joint_states.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME pro450_static_joint_states
)
install(PROGRAMS scripts/pro450_plant_marker_publisher.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME pro450_plant_marker_publisher
)
install(PROGRAMS scripts/pro450_leaf_pipeline.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME pro450_leaf_pipeline
)
ament_package()
