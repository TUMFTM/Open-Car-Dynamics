cmake_minimum_required(VERSION 3.12)
set(PROJECT_NAME ocd_vehicle_nodes_cpp)
project(${PROJECT_NAME})

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

set(NODE_DEPENDENCIES
  ocd_vehicle_model_node_cpp 
  rclcpp_components
  ocd_vehicle_model_cpp
  ocd_drivetrain_wheel_torque_cpp
  ocd_drivetrain_wheel_torque_communication_handler_cpp
  ocd_drivetrain_fx_cpp
  ocd_drivetrain_fx_communication_handler_cpp
  ocd_steering_actuator_pt1_cpp
  ocd_steering_actuator_pt1_communication_handler_cpp
  ocd_vehicle_dynamics_single_track_cpp
  ocd_vehicle_dynamics_double_track_cpp
  ocd_tire_models_cpp
  ocd_aerodynamics_models_cpp
)
# Generate those automatically using the script in tools/create_combinations.py
set(VEHICLE_MODELS 
  WheelTorque__PT1__SingleTrack__MF52__DefaultAerodynamics
  WheelTorque__PT1__SingleTrack__Linear__DefaultAerodynamics
  WheelTorque__PT1__SingleTrack__MF_Simple__DefaultAerodynamics
  WheelTorque__PT1__SingleTrack__MF_Simple_Extended__DefaultAerodynamics
  WheelTorque__PT1__SingleTrack__MF52__RideHeightAerodynamics
  WheelTorque__PT1__SingleTrack__Linear__RideHeightAerodynamics
  WheelTorque__PT1__SingleTrack__MF_Simple__RideHeightAerodynamics
  WheelTorque__PT1__SingleTrack__MF_Simple_Extended__RideHeightAerodynamics
  WheelTorque__PT1__DoubleTrack__MF52__DefaultAerodynamics
  WheelTorque__PT1__DoubleTrack__Linear__DefaultAerodynamics
  WheelTorque__PT1__DoubleTrack__MF_Simple__DefaultAerodynamics
  WheelTorque__PT1__DoubleTrack__MF_Simple_Extended__DefaultAerodynamics
  WheelTorque__PT1__DoubleTrack__MF52__RideHeightAerodynamics
  WheelTorque__PT1__DoubleTrack__Linear__RideHeightAerodynamics
  WheelTorque__PT1__DoubleTrack__MF_Simple__RideHeightAerodynamics
  WheelTorque__PT1__DoubleTrack__MF_Simple_Extended__RideHeightAerodynamics
  Fx__PT1__SingleTrack__MF52__DefaultAerodynamics
  Fx__PT1__SingleTrack__Linear__DefaultAerodynamics
  Fx__PT1__SingleTrack__MF_Simple__DefaultAerodynamics
  Fx__PT1__SingleTrack__MF_Simple_Extended__DefaultAerodynamics
  Fx__PT1__SingleTrack__MF52__RideHeightAerodynamics
  Fx__PT1__SingleTrack__Linear__RideHeightAerodynamics
  Fx__PT1__SingleTrack__MF_Simple__RideHeightAerodynamics
  Fx__PT1__SingleTrack__MF_Simple_Extended__RideHeightAerodynamics
  Fx__PT1__DoubleTrack__MF52__DefaultAerodynamics
  Fx__PT1__DoubleTrack__Linear__DefaultAerodynamics
  Fx__PT1__DoubleTrack__MF_Simple__DefaultAerodynamics
  Fx__PT1__DoubleTrack__MF_Simple_Extended__DefaultAerodynamics
  Fx__PT1__DoubleTrack__MF52__RideHeightAerodynamics
  Fx__PT1__DoubleTrack__Linear__RideHeightAerodynamics
  Fx__PT1__DoubleTrack__MF_Simple__RideHeightAerodynamics
  Fx__PT1__DoubleTrack__MF_Simple_Extended__RideHeightAerodynamics
)

foreach(pkg_name ${NODE_DEPENDENCIES})
  find_package(${pkg_name} REQUIRED)
endforeach()

set(COMPONENT_NAMES)
set(EXECUTABLE_NAMES)

foreach(vehicle_model_name ${VEHICLE_MODELS})
  list(APPEND EXECUTABLE_NAMES ${vehicle_model_name})
  list(APPEND COMPONENT_NAMES ${vehicle_model_name})
endforeach()

add_library(${PROJECT_NAME} SHARED components/components.cpp)
find_package(rclcpp_components REQUIRED)

target_compile_options(${PROJECT_NAME} PRIVATE -fopenmp)

target_include_directories(
  ${PROJECT_NAME}
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(${PROJECT_NAME} ${NODE_DEPENDENCIES})

foreach(vehicle_model_name ${VEHICLE_MODELS})
  rclcpp_components_register_node(
    ${PROJECT_NAME}
    PLUGIN  ${PROJECT_NAME}::${vehicle_model_name}
    EXECUTABLE ${vehicle_model_name}
  )
endforeach()

ament_export_targets(export_${PROJECT_NAME})

    
install(TARGETS ${PROJECT_NAME}
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)


ament_export_dependencies(rclcpp_components ${NODE_DEPENDENCIES}) # other dependencies so client does not need to call find_package on them

ament_package()