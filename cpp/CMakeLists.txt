cmake_minimum_required(VERSION 3.10)
project(horus_cpp_sdk VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Find ROS 2
find_package(ament_cmake QUIET)
if(ament_cmake_FOUND)
    find_package(rclcpp REQUIRED)
    find_package(std_msgs REQUIRED)
    find_package(sensor_msgs REQUIRED)
    find_package(nav_msgs REQUIRED)
    find_package(geometry_msgs REQUIRED)
    set(ROS2_FOUND TRUE)
else()
    message(STATUS "ROS 2 not found. Building without ROS 2 support.")
    set(ROS2_FOUND FALSE)
endif()

# Include directories
include_directories(
    include
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Source files
set(HORUS_SOURCES
    src/core/types.cpp
    src/core/event_bus.cpp
    src/utils/branding.cpp
    # Add other source files as they are implemented
)

# Create library
add_library(${PROJECT_NAME} SHARED ${HORUS_SOURCES})

target_include_directories(${PROJECT_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

if(ROS2_FOUND)
    ament_target_dependencies(${PROJECT_NAME}
        rclcpp
        std_msgs
        sensor_msgs
        nav_msgs
        geometry_msgs
    )
endif()

# Install library
install(TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}Targets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

# Install headers
install(DIRECTORY include/
    DESTINATION include
    FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
)

# Install license
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/../LICENSE
    DESTINATION share/${PROJECT_NAME}
)

# Generate and install CMake config files
include(CMakePackageConfigHelpers)

# Generate package version file
write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
)

# Create config file
configure_package_config_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/Config.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
    INSTALL_DESTINATION lib/cmake/${PROJECT_NAME}
)

# Install CMake config files
install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
    DESTINATION lib/cmake/${PROJECT_NAME}
)

# Install export targets
install(EXPORT ${PROJECT_NAME}Targets
    FILE ${PROJECT_NAME}Targets.cmake
    NAMESPACE horus::
    DESTINATION lib/cmake/${PROJECT_NAME}
)

if(ROS2_FOUND)
    ament_package()
endif()
