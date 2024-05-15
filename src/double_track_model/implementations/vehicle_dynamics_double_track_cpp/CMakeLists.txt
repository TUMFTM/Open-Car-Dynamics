cmake_minimum_required(VERSION 3.12)
set(PROJECT_NAME vehicle_dynamics_double_track_cpp)
project(${PROJECT_NAME})

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(tum_types_cpp REQUIRED)
find_package(param_manager_cpp REQUIRED)
find_package(vehicle_dynamics_helpers_cpp REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/vehicle_dynamics_equations.cpp
  src/vehicle_dynamics_model.cpp
)
target_include_directories(${PROJECT_NAME}
    PUBLIC 
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_17)
target_compile_options(${PROJECT_NAME} PRIVATE -Ofast -fopenmp) #-dndebug -fno-math-errno -fopenmp)

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

ament_target_dependencies(${PROJECT_NAME} Eigen3 tum_types_cpp param_manager_cpp vehicle_dynamics_helpers_cpp)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET) # so client can also call target_link_libraries(template_lib_cpp::template_lib_cpp)
ament_export_dependencies(Eigen3 tum_types_cpp param_manager_cpp vehicle_dynamics_helpers_cpp) # other dependencies so client does not need to call find_package on them

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   ament_lint_auto_find_test_dependencies()

#   ament_add_gtest(unit_tests  # unit tests
#     test/unit_test.cpp 
#   )
#   target_link_libraries(unit_tests ${PROJECT_NAME}) # make our library available to the test executable
# endif()

ament_package()