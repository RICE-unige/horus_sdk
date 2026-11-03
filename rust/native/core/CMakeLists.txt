cmake_minimum_required(VERSION 3.16)
project(horus_native_core LANGUAGES CXX)
option(HORUS_ENABLE_NVENC "Build native FFmpeg/NVENC encoding (requires NVIDIA hardware at runtime)" OFF)
find_package(nlohmann_json 3.9 REQUIRED)
find_package(OpenSSL REQUIRED COMPONENTS Crypto)
find_package(Threads REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(zstd CONFIG REQUIRED)
find_package(assimp CONFIG REQUIRED)
find_package(tinyxml2 CONFIG REQUIRED)
find_package(ZLIB REQUIRED)
find_package(PNG REQUIRED)
find_package(JPEG REQUIRED)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
find_package(WebP REQUIRED)
find_package(TIFF REQUIRED)
add_library(horus_native_core STATIC config.cpp recording.cpp session.cpp cartesian.cpp costmap.cpp description.cpp texture_color.cpp dashboard.cpp api.cpp)
target_sources(horus_native_core PRIVATE
    remote_atlas.cpp remote_atlas_api.cpp
    remote_datasets.cpp remote_datasets_api.cpp
    remote_video.cpp remote_video_api.cpp
    texture_formats.cpp
    remote_gaussian.cpp remote_gaussian_gpu.cpp remote_gaussian_api.cpp
    remote_scene.cpp remote_scene_api.cpp
    remote_encoder.cpp remote_encoder_api.cpp remote_encoder_async.cpp
    mesh_protocol.cpp live_mesh.cpp mesh_page_scheduler.cpp mesh_source.cpp mesh_source_api.cpp remote_protocol.cpp remote_gpu.cpp remote_frame.cpp remote_dynamic.cpp remote_frame_api.cpp remote_stereo.cpp remote_stereo_api.cpp
    vendor/meshoptimizer/allocator.cpp vendor/meshoptimizer/indexgenerator.cpp
    vendor/meshoptimizer/simplifier.cpp vendor/meshoptimizer/vcacheoptimizer.cpp
    vendor/meshoptimizer/vfetchoptimizer.cpp)
set_target_properties(horus_native_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_compile_features(horus_native_core PUBLIC cxx_std_20)
target_include_directories(horus_native_core PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
target_include_directories(horus_native_core PRIVATE vendor/mcap/include vendor/meshoptimizer)
target_compile_definitions(horus_native_core PRIVATE MCAP_COMPRESSION_NO_LZ4)
if(TARGET zstd::libzstd_shared)
  set(HORUS_ZSTD_TARGET zstd::libzstd_shared)
elseif(TARGET zstd::libzstd_static)
  set(HORUS_ZSTD_TARGET zstd::libzstd_static)
else()
  set(HORUS_ZSTD_TARGET zstd::zstd)
endif()
target_link_libraries(horus_native_core PUBLIC nlohmann_json::nlohmann_json PRIVATE Eigen3::Eigen OpenSSL::Crypto Threads::Threads ${HORUS_ZSTD_TARGET} assimp::assimp tinyxml2::tinyxml2 ZLIB::ZLIB PNG::PNG JPEG::JPEG)
target_link_libraries(horus_native_core PRIVATE ${CMAKE_DL_LIBS})
target_link_libraries(horus_native_core PRIVATE WebP::webp TIFF::TIFF)
if(HORUS_ENABLE_NVENC)
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(HORUS_AV REQUIRED IMPORTED_TARGET libavcodec>=60 libavutil>=58)
  target_compile_definitions(horus_native_core PRIVATE HORUS_ENABLE_NVENC=1)
  target_link_libraries(horus_native_core PRIVATE PkgConfig::HORUS_AV)
endif()
install(TARGETS horus_native_core EXPORT horus_native_coreTargets ARCHIVE DESTINATION lib)
install(EXPORT horus_native_coreTargets FILE horus_native_coreTargets.cmake NAMESPACE horus_native:: DESTINATION lib/cmake/horus_native_core)
install(DIRECTORY include/ DESTINATION include)
install(FILES api.h remote_gpu.h remote_frame.h remote_stereo.h remote_encoder.h remote_scene.h mesh_source_api.h DESTINATION include/horus/native/core)
install(FILES remote_gaussian.h remote_gaussian_backend.h DESTINATION include/horus/native/core)
install(FILES remote_video.h remote_video_backend.h DESTINATION include/horus/native/core)
install(FILES remote_datasets.h DESTINATION include/horus/native/core)
install(FILES remote_atlas.h DESTINATION include/horus/native/core)
install(FILES vendor/mcap/LICENSE DESTINATION share/horus_native_core/mcap)
install(FILES vendor/happly/LICENSE vendor/happly/PROVENANCE.md DESTINATION share/horus_native_core/happly)
install(FILES vendor/stb/LICENSE vendor/stb/PROVENANCE.md DESTINATION share/horus_native_core/stb)
install(FILES cmake/FindWebP.cmake DESTINATION lib/cmake/horus_native_core)
install(FILES vendor/meshoptimizer/LICENSE.md vendor/meshoptimizer/PROVENANCE.md DESTINATION share/horus_native_core/meshoptimizer)
