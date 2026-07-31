cmake_minimum_required(VERSION 3.8)
project(pro450_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config meshes urdf DESTINATION share/${PROJECT_NAME})
install(FILES UPSTREAM_LICENSE DESTINATION share/${PROJECT_NAME})

ament_package()
