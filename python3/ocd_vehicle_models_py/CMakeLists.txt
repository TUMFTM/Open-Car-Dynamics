cmake_minimum_required(VERSION 3.5)
project(ocd_vehicle_models_py)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Figure out Python3 debug/release before anything else can find_package it
if(WIN32 AND CMAKE_BUILD_TYPE STREQUAL "Debug")
  find_package(python_cmake_module REQUIRED)
  find_package(PythonExtra REQUIRED)

  # Force FindPython3 to use the debug interpreter where ROS 2 expects it
  set(Python3_EXECUTABLE "${PYTHON_EXECUTABLE_DEBUG}")
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ament_cmake_ros REQUIRED)

# Find python before pybind11
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)

find_package(pybind11_vendor REQUIRED)
find_package(pybind11 REQUIRED)

set (MODEL_DEPS 
  tum_types_cpp
  ocd_types_cpp
  ocd_aerodynamics_models_cpp
  ocd_drivetrain_fx_cpp
  ocd_steering_actuator_pt1_cpp
  ocd_tire_models_cpp
  ocd_drivetrain_wheel_torque_cpp
  ocd_vehicle_dynamics_double_track_cpp
  ocd_vehicle_dynamics_single_track_cpp
  ocd_vehicle_model_cpp
)

foreach(pkg_name ${MODEL_DEPS})
  find_package(${pkg_name} REQUIRED)
endforeach()

# region target cpp bindings
ament_python_install_package(${PROJECT_NAME})

# region create target for cpp binding 
pybind11_add_module(_ocd_vehicle_models_py SHARED
  src/_cpp_binding.cpp
)

target_include_directories(_ocd_vehicle_models_py
  PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(_ocd_vehicle_models_py PUBLIC
  ${MODEL_DEPS}
)
ament_export_dependencies(${MODEL_DEPS})

 # Install cython modules as sub-modules of the project
install(
  TARGETS
    _ocd_vehicle_models_py
  DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}"
)
#endregion

ament_package()
