cmake_minimum_required(VERSION 3.8)
project(horus_unity_bridge)

# Compiler settings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3 -march=native)
endif()

# C++17 standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(rosidl_default_runtime REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)
find_package(rosidl_typesupport_introspection_cpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(example_interfaces REQUIRED)

# Find nlohmann_json (header-only library)
find_package(nlohmann_json 3.2.0 QUIET)
if(NOT nlohmann_json_FOUND)
  # If not found via find_package, try to use system install
  find_path(NLOHMANN_JSON_INCLUDE_DIR nlohmann/json.hpp
    HINTS /usr/include /usr/local/include)
  if(NLOHMANN_JSON_INCLUDE_DIR)
    message(STATUS "Found nlohmann/json: ${NLOHMANN_JSON_INCLUDE_DIR}")
  else()
    message(FATAL_ERROR "nlohmann_json not found. Install with: sudo apt-get install nlohmann-json3-dev")
  endif()
endif()

# Include directories
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
)

# Source files
set(BRIDGE_SOURCES
  src/main.cpp
  src/unity_bridge_node.cpp
  src/protocol_handler.cpp
  src/connection_manager.cpp
  src/message_router.cpp
  src/topic_manager.cpp
  src/service_manager.cpp
)

# Unity bridge executable
add_executable(horus_unity_bridge_node ${BRIDGE_SOURCES})

ament_target_dependencies(horus_unity_bridge_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2_msgs
  rosidl_default_runtime
  rosidl_typesupport_cpp
  rosidl_typesupport_introspection_cpp
  std_srvs
  example_interfaces
)

# Link pthread for threading support
target_link_libraries(horus_unity_bridge_node pthread)

# Install targets
install(TARGETS horus_unity_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
