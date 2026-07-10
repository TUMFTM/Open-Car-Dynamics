# Copyright 2026 Simon Sagmeister
cmake_minimum_required(VERSION 3.12)

set(PROJECT_NAME ocd_external_influence_aggregator_cpp)
project(${PROJECT_NAME})

set(NODE_DEPENDENCIES
  rclcpp
  rclcpp_components
  std_msgs
  geometry_msgs
  tum_msgs
  ocd_interfaces
  tum_ros_helpers_cpp
)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
foreach(pkg_name ${NODE_DEPENDENCIES})
  find_package(${pkg_name} REQUIRED)
endforeach()

# The node library also carries the component registration, which produces both
# the composable component and a standalone `external_influence_aggregator_node`
# executable.
add_library(${PROJECT_NAME} SHARED
  src/external_influence_aggregator_node.cpp
  components/external_influence_aggregator_component.cpp)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(${PROJECT_NAME} ${NODE_DEPENDENCIES})

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "tam::ocd::ExternalInfluenceAggregatorNode"
  EXECUTABLE external_influence_aggregator_node)

install(
  DIRECTORY include/
  DESTINATION include
)
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${NODE_DEPENDENCIES})
ament_package()
