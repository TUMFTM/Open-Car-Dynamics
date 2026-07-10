cmake_minimum_required(VERSION 3.12)
set(PROJECT_NAME ocd_vehicle_model_node_cpp)
project(${PROJECT_NAME})

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

set(NODE_DEPENDENCIES
    rclcpp
    tf2
    param_management_cpp
    param_management_ros2_integration_cpp
    tsl_logger_cpp
    tsl_ros2_publisher_cpp
    nav_msgs
    tum_msgs
    ocd_interfaces
    ocd_types_cpp
    tum_ros_helpers_cpp
    tum_type_conversions_ros_cpp
    ocd_vehicle_model_base_cpp
    ocd_communication_handler_base_cpp
)

foreach(pkg_name ${NODE_DEPENDENCIES})
  find_package(${pkg_name} REQUIRED)
endforeach()

add_library(${PROJECT_NAME} INTERFACE)

target_include_directories(
  ${PROJECT_NAME}
  INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# target_compile_options(${PROJECT_NAME} INTERFACE -fopenmp)


ament_target_dependencies(${PROJECT_NAME} INTERFACE ${NODE_DEPENDENCIES})


ament_export_targets(export_${PROJECT_NAME})

    
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

ament_export_dependencies(${NODE_DEPENDENCIES}) # other dependencies so client does not need to call find_package on them

ament_package()