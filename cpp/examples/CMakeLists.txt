add_executable(sdk_registration_demo sdk_registration_demo.cpp)
target_link_libraries(sdk_registration_demo PRIVATE horus_cpp_sdk)

add_executable(fake_tf_publisher fake_tf_publisher.cpp)
target_link_libraries(fake_tf_publisher PRIVATE horus_cpp_sdk)

add_executable(e2e_registration_check e2e_registration_check.cpp)
target_link_libraries(e2e_registration_check PRIVATE horus_cpp_sdk)

install(TARGETS
  sdk_registration_demo
  fake_tf_publisher
  e2e_registration_check
  RUNTIME DESTINATION bin/examples
)

