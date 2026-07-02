cmake_minimum_required(VERSION 3.18)

project(lib_open_car_dynamics_cpp VERSION 1.0 LANGUAGES CXX)

# Get the absolute path to the repository root
get_filename_component(REPO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/.." ABSOLUTE)

find_package(Eigen3 REQUIRED)

# ------------------------------------------------------------------------------
# library
# ------------------------------------------------------------------------------
add_library(${PROJECT_NAME} SHARED)
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

# ------------------------------------------------------------------------------
# List of packages - using Absolute Paths
# ------------------------------------------------------------------------------
file(GLOB LIB_PKGS RELATIVE "${REPO_ROOT}" "../lib/*")
set(PKG_ABS_DIRS "")

foreach(PKG_REL ${LIB_PKGS})
    if(IS_DIRECTORY "${REPO_ROOT}/${PKG_REL}")
        list(APPEND PKG_ABS_DIRS "${REPO_ROOT}/${PKG_REL}")
    endif()
endforeach()

# Manually add the extra dependencies with absolute paths
set(EXTRA_DEPS
    "submodules/tam__common/submodules/parameter_management/param_management_cpp"
    "submodules/tam__common/submodules/time_series_logging/tsl_logger_cpp"
    "submodules/tam__common/tum_helpers_cpp"
    "submodules/tam__common/tum_types_cpp"
    "submodules/tam__common/vendor_pkgs/matplotlib_cpp"
)

foreach(DEP ${EXTRA_DEPS})
    list(APPEND PKG_ABS_DIRS "${REPO_ROOT}/${DEP}")
endforeach()

# ------------------------------------------------------------------------------
# Include directories and Source Gathering
# ------------------------------------------------------------------------------
set(SRC_FILES "")
foreach(PKG_PATH ${PKG_ABS_DIRS})
    target_include_directories(${PROJECT_NAME}
        PUBLIC
            $<BUILD_INTERFACE:${PKG_PATH}/include>
            $<INSTALL_INTERFACE:include>
    )
    
    file(GLOB_RECURSE PKG_SRC "${PKG_PATH}/src/*.cpp")
    list(APPEND SRC_FILES ${PKG_SRC})
endforeach()

target_sources(${PROJECT_NAME} PRIVATE ${SRC_FILES})

# ... (Link dependencies and compiler settings stay the same) ...
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_20)
target_compile_definitions(${PROJECT_NAME} PRIVATE BUILDING_CURVILINEAR_COSY)

# ------------------------------------------------------------------------------
# Installation
# ------------------------------------------------------------------------------
set(CMAKE_INSTALL_PREFIX "${CMAKE_CURRENT_SOURCE_DIR}/install" CACHE PATH "Install path" FORCE)

install(TARGETS ${PROJECT_NAME} EXPORT ${PROJECT_NAME}Targets LIBRARY DESTINATION lib)
install(EXPORT ${PROJECT_NAME}Targets NAMESPACE ${PROJECT_NAME}:: DESTINATION lib/cmake/${PROJECT_NAME})

foreach(PKG_PATH ${PKG_ABS_DIRS})  
    if(EXISTS "${PKG_PATH}/include/")
        install(DIRECTORY "${PKG_PATH}/include/" DESTINATION include)
    endif()
endforeach()

# ------------------------------------------------------------------------------
# Config and Version files
# ------------------------------------------------------------------------------
include(CMakePackageConfigHelpers)

write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)

# IMPORTANT: Check if your file is 'open_car_dynamics_cppConfig.cmake.in' 
# or 'lib_open_car_dynamics_cppConfig.cmake.in'. 
# Based on your first post, it's 'open_car_dynamics_cppConfig.cmake.in'
configure_package_config_file(
    "${REPO_ROOT}/cmake/open_car_dynamics_cppConfig.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
    INSTALL_DESTINATION lib/cmake/${PROJECT_NAME}
)

install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
    DESTINATION lib/cmake/${PROJECT_NAME}
)

# ------------------------------------------------------------------------------
# setup.sh
# ------------------------------------------------------------------------------
configure_file(
    "${REPO_ROOT}/cmake/setup.sh.in"
    "${CMAKE_CURRENT_BINARY_DIR}/setup.sh"
    @ONLY
)

install(FILES "${CMAKE_CURRENT_BINARY_DIR}/setup.sh" DESTINATION . 
        PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)