cmake_minimum_required(VERSION 3.12)
set(PROJECT_NAME tum_type_conversions_ros_cpp)
project(${PROJECT_NAME})

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tum_types_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tum_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/tum_type_conversions.cpp
  src/orientation.cpp
)

target_include_directories(${PROJECT_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(
  ${PROJECT_NAME} 
  rclcpp
  tf2
  diagnostic_msgs
  nav_msgs
  geometry_msgs
  tum_msgs
  sensor_msgs
  tum_types_cpp
)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET) # so client can also call target_link_libraries(template_lib_cpp::template_lib_cpp)
ament_export_dependencies(
  rclcpp
  tf2
  nav_msgs
  tum_types_cpp
  diagnostic_msgs
  geometry_msgs
  tum_msgs
  sensor_msgs
  tum_types_cpp
) # other dependencies so client does not need to call find_package on them


ament_package()