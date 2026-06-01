set(HORUS_REGISTRATION_EXAMPLES
  sdk_registration_demo
  ops_registration
  flat_robot_registration
  drone_registration
  legged_registration
  stereo_registration
  semantic_perception_registration
  robot_description_registration
  robot_description_compass_registration
  occupancy_map_registration
  pointcloud_map_registration
  mesh_map_registration
  octomap_registration
  global_maps_registration
  gaussian_splat_fixture_registration
  carter_registration
  unitree_go1_registration
  uav_sim_horus_registration
)

foreach(example_target IN LISTS HORUS_REGISTRATION_EXAMPLES)
  add_executable(${example_target} ${example_target}.cpp)
  target_link_libraries(${example_target} PRIVATE horus_cpp_sdk)
endforeach()

add_executable(fake_tf_publisher fake_tf_publisher.cpp)
target_link_libraries(fake_tf_publisher PRIVATE horus_cpp_sdk)

add_executable(e2e_registration_check e2e_registration_check.cpp)
target_link_libraries(e2e_registration_check PRIVATE horus_cpp_sdk)

install(TARGETS
  ${HORUS_REGISTRATION_EXAMPLES}
  fake_tf_publisher
  e2e_registration_check
  RUNTIME DESTINATION bin/examples
)
