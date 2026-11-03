cmake_minimum_required(VERSION 3.24)
project(horus_gaussian_backend LANGUAGES CXX CUDA)
set(GSPLAT_SOURCE_DIR "" CACHE PATH "Explicit gsplat 1.5.3 source directory containing cuda/include/Ops.h")
if(NOT EXISTS "${GSPLAT_SOURCE_DIR}/cuda/include/Ops.h")
  message(FATAL_ERROR "Set GSPLAT_SOURCE_DIR to the gsplat 1.5.3 package directory")
endif()
find_package(Torch REQUIRED)
find_package(CUDAToolkit REQUIRED)
set(src "${GSPLAT_SOURCE_DIR}/cuda/csrc")
file(GLOB cuda_sources CONFIGURE_DEPENDS "${src}/Projection*.cu" "${src}/Rasterize*.cu")
add_library(horus_gaussian SHARED gaussian_backend.cpp ${cuda_sources}
  "${src}/Projection.cpp" "${src}/Rasterization.cpp"
  "${src}/Intersect.cpp" "${src}/IntersectTile.cu"
  "${src}/SphericalHarmonics.cpp" "${src}/SphericalHarmonicsCUDA.cu")
target_compile_features(horus_gaussian PRIVATE cxx_std_17)
set_target_properties(horus_gaussian PROPERTIES CUDA_STANDARD 17 POSITION_INDEPENDENT_CODE ON)
target_include_directories(horus_gaussian PRIVATE "${GSPLAT_SOURCE_DIR}/cuda/include"
  "${src}" "${src}/third_party/glm")
separate_arguments(torch_flags NATIVE_COMMAND "${TORCH_CXX_FLAGS}")
target_compile_options(horus_gaussian PRIVATE ${torch_flags})
target_compile_options(horus_gaussian PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:--expt-relaxed-constexpr>)
target_link_libraries(horus_gaussian PRIVATE ${TORCH_LIBRARIES} CUDA::cudart)
install(TARGETS horus_gaussian LIBRARY DESTINATION lib)
