cmake_minimum_required(VERSION 3.5)
project(leaf_manipulation_sim)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY
  urdf
  launch
  config
  worlds
  models
  rviz
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  urdf/sanitize_urdf_for_gazebo.py
  DESTINATION share/${PROJECT_NAME}/urdf
)

install(PROGRAMS
  leaf_manipulation_sim/static_joint_states.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME static_joint_states
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
