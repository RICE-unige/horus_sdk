cmake_minimum_required(VERSION 3.16)
project(horus_cpp_sdk VERSION 0.1.0 LANGUAGES CXX)

option(HORUS_BUILD_EXAMPLES "Build C++ examples" ON)
option(HORUS_BUILD_TESTS "Build C++ tests" ON)
option(HORUS_BUILD_BENCHMARKS "Build C++ benchmark binaries" ON)
option(HORUS_ENABLE_ROS2 "Enable ROS2 integration when dependencies are available" ON)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(ROS2_FOUND FALSE)
if(HORUS_ENABLE_ROS2)
  find_package(ament_cmake QUIET)
  if(ament_cmake_FOUND)
    find_package(rclcpp QUIET)
    find_package(std_msgs QUIET)
    find_package(sensor_msgs QUIET)
    find_package(nav_msgs QUIET)
    find_package(geometry_msgs QUIET)
  endif()

  if(ament_cmake_FOUND AND rclcpp_FOUND AND std_msgs_FOUND AND sensor_msgs_FOUND
     AND nav_msgs_FOUND AND geometry_msgs_FOUND)
    set(ROS2_FOUND TRUE)
  endif()
endif()

add_library(${PROJECT_NAME} SHARED
  src/client.cpp
  src/core/types.cpp
  src/core/event_bus.cpp
  src/core/topic_map.cpp
  src/bridge/ros2.cpp
  src/bridge/unity_tcp.cpp
  src/bridge/robot_registry.cpp
  src/robot/robot.cpp
  src/robot/sensors.cpp
  src/robot/dataviz.cpp
  src/robot/status.cpp
  src/robot/task.cpp
  src/robot/teleop.cpp
  src/plugins/rosbot.cpp
  src/topics.cpp
  src/utils/branding.cpp
  src/utils/logging.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

if(ROS2_FOUND)
  target_compile_definitions(${PROJECT_NAME} PUBLIC HORUS_ROS2_ENABLED=1)
  ament_target_dependencies(${PROJECT_NAME}
    rclcpp
    std_msgs
    sensor_msgs
    nav_msgs
    geometry_msgs
  )
else()
  if(HORUS_ENABLE_ROS2)
    message(STATUS "ROS 2 dependencies not found. Building in no-ROS mode.")
  else()
    message(STATUS "HORUS_ENABLE_ROS2=OFF. Building in no-ROS mode.")
  endif()
endif()

add_library(horus::horus_cpp_sdk ALIAS ${PROJECT_NAME})

if(HORUS_BUILD_EXAMPLES)
  add_subdirectory(examples)
endif()

if(HORUS_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()

if(HORUS_BUILD_BENCHMARKS)
  add_subdirectory(benchmarks)
endif()

install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(DIRECTORY include/
  DESTINATION include
  FILES_MATCHING
  PATTERN "*.hpp"
  PATTERN "*.h"
)

install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/../LICENSE
  DESTINATION share/${PROJECT_NAME}
)

include(CMakePackageConfigHelpers)
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY AnyNewerVersion
)

configure_package_config_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/Config.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
  INSTALL_DESTINATION lib/cmake/${PROJECT_NAME}
)

install(FILES
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
  DESTINATION lib/cmake/${PROJECT_NAME}
)

install(EXPORT ${PROJECT_NAME}Targets
  FILE ${PROJECT_NAME}Targets.cmake
  NAMESPACE horus::
  DESTINATION lib/cmake/${PROJECT_NAME}
)

if(HORUS_ENABLE_ROS2 AND ROS2_FOUND AND ament_cmake_FOUND)
  ament_package()
endif()
