cmake_minimum_required(VERSION 3.5)
project(leaf_manipulation_sim)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)

ament_python_install_package(${PROJECT_NAME})

add_executable(circle_motion_demo src/circle_motion_demo.cpp)
ament_target_dependencies(circle_motion_demo
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  nav_msgs
  moveit_ros_planning_interface
  moveit_msgs
)

add_executable(move_to_pose_grasp_demo src/move_to_pose_grasp_demo.cpp)
ament_target_dependencies(move_to_pose_grasp_demo
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  nav_msgs
  moveit_ros_planning_interface
  moveit_msgs
)

install(TARGETS
  circle_motion_demo
  move_to_pose_grasp_demo
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  urdf
  launch
  config
  worlds
  models
  meshes
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

install(PROGRAMS
  leaf_manipulation_sim/initial_controller_commands.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME initial_controller_commands
)

install(PROGRAMS
  leaf_manipulation_sim/prompt_pose_grasp_demo.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME prompt_pose_grasp_demo
)

install(PROGRAMS
  leaf_manipulation_sim/leaf_pose_adapter.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME leaf_pose_adapter
)

install(PROGRAMS
  leaf_manipulation_sim/mock_leaf_pose_publisher.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME mock_leaf_pose_publisher
)

install(PROGRAMS
  leaf_manipulation_sim/plant_marker_publisher.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME plant_marker_publisher
)

install(PROGRAMS
  leaf_manipulation_sim/contact_bridge_adapter.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME contact_bridge_adapter
)

install(PROGRAMS
  leaf_manipulation_sim/leaf_grasp_demo.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME leaf_grasp_demo
)

install(PROGRAMS
  leaf_manipulation_sim/leaf_mtc_pinch_demo.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME leaf_mtc_pinch_demo
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
