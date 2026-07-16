# Copyright 2026 Simon Sagmeister
cmake_minimum_required(VERSION 3.5)
project(ocd_interfaces)

# Package dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tum_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Get all msg files
file(GLOB msgFiles RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/msg/*.msg)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msgFiles}
  DEPENDENCIES std_msgs geometry_msgs tum_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
