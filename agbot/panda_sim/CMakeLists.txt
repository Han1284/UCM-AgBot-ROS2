cmake_minimum_required(VERSION 3.5)
project(panda_sim)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  meshes
  rviz
  urdf
  worlds
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/panda_static_joint_states.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME panda_static_joint_states
)

install(PROGRAMS
  scripts/sanitize_urdf_for_gazebo.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME sanitize_urdf_for_gazebo
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
