set(NODE_DEPENDENCIES 
  rclcpp
  vehicle_model_double_track_cpp
  tum_types_cpp
  ros_param_helpers_cpp
  ros_debug_helpers_cpp
  tum_type_conversions_ros_cpp
  nav_msgs
  tum_msgs
  tf2
  autoware_auto_control_msgs
  autoware_auto_vehicle_msgs
  tum_sim_types_cpp
)
set(VEHICLE_MODELS 
  vehicle_model_double_track_cpp
)

foreach(pkg_name ${VEHICLE_MODELS})
  find_package(${pkg_name} REQUIRED)
endforeach()
foreach(pkg_name ${NODE_DEPENDENCIES})
  find_package(${pkg_name} REQUIRED)
endforeach()

# Iterate to create all the targets
# Simulink node
foreach(vehicle_model_name ${VEHICLE_MODELS})
  set(node_executable_name ${vehicle_model_name}_node)
  add_executable(${node_executable_name} executables/${node_executable_name}.cpp vehicle_model_node.cpp)
  target_include_directories(${node_executable_name} PUBLIC  
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>
      $<INSTALL_INTERFACE:include>
  )
  target_compile_options(${node_executable_name} PRIVATE -Ofast -fopenmp) #-dndebug -fno-math-errno -fopenmp)

  ament_target_dependencies(${node_executable_name} ${NODE_DEPENDENCIES} ${vehicle_model_name})

  install(TARGETS ${node_executable_name} DESTINATION lib/${PROJECT_NAME})
  install(
    DIRECTORY ../include/
    DESTINATION include
  )
  
endforeach()

