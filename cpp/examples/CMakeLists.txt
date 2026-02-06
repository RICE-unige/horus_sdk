cmake_minimum_required(VERSION 3.10)
project(horus_cpp_examples)

# Find the horus_cpp_sdk package
find_package(horus_cpp_sdk REQUIRED)

# Registration demo example
add_executable(sdk_registration_demo sdk_registration_demo.cpp)
target_link_libraries(sdk_registration_demo PRIVATE horus::horus_cpp_sdk)

# Install examples (optional)
install(TARGETS sdk_registration_demo
    RUNTIME DESTINATION bin/examples
)
