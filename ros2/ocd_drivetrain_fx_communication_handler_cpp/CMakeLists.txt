cmake_minimum_required(VERSION 3.12)
set(PROJECT_NAME ocd_drivetrain_fx_communication_handler_cpp)
project(${PROJECT_NAME})

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

set(DEPENDENCIES
    rclcpp  
    ocd_communication_handler_base_cpp
    ocd_helpers_cpp
    ocd_vehicle_model_node_cpp
    param_management_cpp
    param_management_ros2_integration_cpp
    std_msgs
    tum_types_cpp
    ocd_types_cpp
    tum_ros_helpers_cpp
    ocd_drivetrain_fx_cpp
)

foreach(pkg_name ${DEPENDENCIES})
  find_package(${pkg_name} REQUIRED)
endforeach()

add_library(${PROJECT_NAME} SHARED src/comm_handler.cpp)

target_include_directories(
  ${PROJECT_NAME}
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_options(${PROJECT_NAME} PRIVATE -fopenmp)

ament_target_dependencies(${PROJECT_NAME} ${DEPENDENCIES})


ament_export_targets(export_${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

install(DIRECTORY include/
        DESTINATION include/)

ament_export_dependencies(${DEPENDENCIES}) # other dependencies so client does not need to call find_package on them

ament_package()