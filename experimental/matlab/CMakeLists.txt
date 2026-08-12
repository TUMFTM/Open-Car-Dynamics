cmake_minimum_required(VERSION 3.18)

project(ocd_matlab_experimental LANGUAGES CXX)

option(OCD_MATLAB_BUILD_MEX "Build the MATLAB MEX module" ON)
option(OCD_MATLAB_BUILD_TESTS "Build the MATLAB-independent compile and smoke tests" ON)

get_filename_component(OCD_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)

# Build the standalone aggregate library in the same build tree. This keeps the
# experimental binding independent of ROS 2/ament and avoids requiring a global
# Open Car Dynamics installation.
if(NOT TARGET lib_open_car_dynamics_cpp)
  add_subdirectory(
    "${OCD_ROOT}/cmake_build"
    "${CMAKE_CURRENT_BINARY_DIR}/open_car_dynamics"
    EXCLUDE_FROM_ALL
  )
endif()

add_library(ocd_matlab_adapter STATIC src/model.cpp)
target_include_directories(
  ocd_matlab_adapter
  PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include"
)
target_link_libraries(ocd_matlab_adapter PUBLIC lib_open_car_dynamics_cpp)
target_compile_features(ocd_matlab_adapter PUBLIC cxx_std_20)
set_target_properties(ocd_matlab_adapter PROPERTIES POSITION_INDEPENDENT_CODE ON)
if(MSVC)
  target_compile_options(ocd_matlab_adapter PRIVATE /W4 /WX)
else()
  target_compile_options(ocd_matlab_adapter PRIVATE -Wall -Wextra -Wpedantic -Werror)
endif()

if(OCD_MATLAB_BUILD_MEX)
  find_package(Matlab REQUIRED)

  matlab_add_mex(
    NAME ocd_matlab
    SRC src/gateway.cpp
    DOCUMENTATION src/ocd_matlab.m
    LINK_TO ocd_matlab_adapter
    R2018a
  )
  set_target_properties(
    ocd_matlab
    PROPERTIES
      LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/matlab"
      RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/matlab"
  )
  if(MSVC)
    target_compile_options(ocd_matlab PRIVATE /W4 /WX)
  else()
    target_compile_options(ocd_matlab PRIVATE -Wall -Wextra -Wpedantic -Werror)
  endif()
endif()

if(OCD_MATLAB_BUILD_TESTS)
  enable_testing()

  add_executable(
    ocd_matlab_factory_smoke_test
    tests/factory_smoke_test.cpp
  )
  target_link_libraries(ocd_matlab_factory_smoke_test PRIVATE ocd_matlab_adapter)
  if(MSVC)
    target_compile_options(ocd_matlab_factory_smoke_test PRIVATE /W4 /WX)
  else()
    target_compile_options(ocd_matlab_factory_smoke_test PRIVATE -Wall -Wextra -Wpedantic -Werror)
  endif()
  add_test(NAME ocd_matlab_factory_smoke_test COMMAND ocd_matlab_factory_smoke_test)

  # MATLAB is unavailable on many development and CI machines. Compile the
  # exact gateway source against a deliberately minimal API declaration so
  # ordinary C++ errors in the gateway are still caught without MATLAB. A real
  # MEX build remains the authoritative check for MathWorks header compatibility.
  add_library(ocd_matlab_mex_compile_check OBJECT src/gateway.cpp)
  target_include_directories(
    ocd_matlab_mex_compile_check
    PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/tests/mex_stub"
  )
  target_link_libraries(ocd_matlab_mex_compile_check PRIVATE ocd_matlab_adapter)
  if(MSVC)
    target_compile_options(ocd_matlab_mex_compile_check PRIVATE /W4 /WX)
  else()
    target_compile_options(ocd_matlab_mex_compile_check PRIVATE -Wall -Wextra -Wpedantic -Werror)
  endif()
endif()

