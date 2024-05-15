cmake_minimum_required(VERSION 3.12)
set(PROJECT_NAME vehicle_model_double_track_cpp)
project(${PROJECT_NAME})

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


set(MODEL_DEPENDENCIES 
  vehicle_model_base_cpp
  param_manager_cpp
  vehicle_dynamics_double_track_cpp
  simple_drivetrain_cpp
  vehicle_dynamics_helpers_cpp
)

find_package(ament_cmake)
foreach(pkg_name ${MODEL_DEPENDENCIES})
  find_package(${pkg_name} REQUIRED)
endforeach()


add_library(${PROJECT_NAME} SHARED
  src/double_track_model.cpp
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

ament_target_dependencies(${PROJECT_NAME} 
  ${MODEL_DEPENDENCIES}
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET) # so client can also call target_link_libraries(template_lib_cpp::template_lib_cpp)
ament_export_dependencies(${MODEL_DEPENDENCIES}) # other dependencies so client does not need to call find_package on them

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   ament_lint_auto_find_test_dependencies()

#   ament_add_gtest(unit_tests  # unit tests
#     test/unit_test.cpp 
#   )
#   target_link_libraries(unit_tests ${PROJECT_NAME}) # make our library available to the test executable
# endif()

ament_package()