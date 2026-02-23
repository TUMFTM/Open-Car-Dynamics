cmake_minimum_required(VERSION 3.12)
set(PROJECT_NAME ocd_tire_models_cpp)
project(${PROJECT_NAME})

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tum_helpers_cpp REQUIRED)
find_package(param_management_cpp REQUIRED)
find_package(ocd_tire_model_base_cpp REQUIRED)

# -- Library target --------------------------------------------------------
add_library(${PROJECT_NAME} SHARED
  src/linear.cpp
  src/mf_simple.cpp
  src/mf_simple_extended.cpp
  src/mf_52.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# C++ standard and compile options
target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_17)
target_compile_options(${PROJECT_NAME} PRIVATE -fopenmp)

# Link / dependency info for the library (so downstream targets see deps)
ament_target_dependencies(${PROJECT_NAME}
  Eigen3
  tum_types_cpp
  tum_helpers_cpp
  param_management_cpp
  ocd_tire_model_base_cpp
)

# Install library & headers
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
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 tum_types_cpp param_management_cpp tum_helpers_cpp)

# ament package boilerplate
ament_package()
