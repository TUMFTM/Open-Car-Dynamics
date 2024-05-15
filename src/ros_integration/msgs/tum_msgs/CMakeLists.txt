cmake_minimum_required(VERSION 3.5)
project(tum_msgs)

# Default to C++14
set(CMAKE_CXX_STANDARD 14)

# Package dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)


# Get all msg files
file(GLOB msgFiles RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/msg/*.msg)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msgFiles}
  DEPENDENCIES builtin_interfaces
  DEPENDENCIES std_msgs
  DEPENDENCIES geometry_msgs
 )

ament_package()
