#include <cuda_runtime.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <new>
#include <string>

namespace
{
constexpr int kMaximumViews = 2;
thread_local std::string g_last_error;

struct Renderer
{
  std::size_t vertex_count = 0;
  std::size_t face_count = 0;
  float3 * vertices = nullptr;
  uint3 * faces = nullptr;
  float2 * uvs = nullptr;
  uchar3 * tints = nullptr;
  uint4 * texture_rects = nullptr;
  uchar3 * texture_atlas = nullptr;
  int texture_width = 0;
  int texture_height = 0;
  float * camera_positions = nullptr;
  float * world_to_camera = nullptr;
  float * projections = nullptr;
  unsigned long long * winners = nullptr;
  uchar3 * output_colors = nullptr;
  float * output_depth = nullptr;
  std::size_t output_capacity = 0;
  cudaEvent_t begin = nullptr;
  cudaEvent_t end = nullptr;
};

bool check_cuda(cudaError_t result, const char * operation)
{
  if (result == cudaSuccess) {
    return true;
  }
  g_last_error = std::string(operation) + ": " + cudaGetErrorString(result);
  return false;
}

__device__ float edge(float ax, float ay, float bx, float by, float px, float py)
{
  return (px - ax) * (by - ay) - (py - ay) * (bx - ax);
}

__device__ float3 transform_vertex(
  const float3 vertex,
  const float * position,
  const float * matrix)
{
  const float px = vertex.x - position[0];
  const float py = vertex.y - position[1];
  const float pz = vertex.z - position[2];
  return make_float3(
    matrix[0] * px + matrix[1] * py + matrix[2] * pz,
    matrix[3] * px + matrix[4] * py + matrix[5] * pz,
    matrix[6] * px + matrix[7] * py + matrix[8] * pz);
}

__device__ float3 interpolate_at_z(const float3 a, const float3 b, float z)
{
  const float denominator = b.z - a.z;
  const float t = fabsf(denominator) > 1e-8f ? (z - a.z) / denominator : 0.0f;
  return make_float3(
    a.x + (b.x - a.x) * t,
    a.y + (b.y - a.y) * t,
    z);
}

__device__ int clip_polygon_z(
  const float3 * input,
  int input_count,
  float plane_z,
  bool keep_greater,
  float3 * output)
{
  if (input_count <= 0) {
    return 0;
  }
  int output_count = 0;
  float3 previous = input[input_count - 1];
  bool previous_inside = keep_greater
    ? previous.z >= plane_z
    : previous.z <= plane_z;
  for (int index = 0; index < input_count; ++index) {
    const float3 current = input[index];
    const bool current_inside = keep_greater
      ? current.z >= plane_z
      : current.z <= plane_z;
    if (current_inside != previous_inside) {
      output[output_count++] = interpolate_at_z(previous, current, plane_z);
    }
    if (current_inside) {
      output[output_count++] = current;
    }
    previous = current;
    previous_inside = current_inside;
  }
  return output_count;
}

__device__ void rasterize_projected_triangle(
  const float3 a,
  const float3 b,
  const float3 c,
  const float * projection,
  int width,
  int height,
  unsigned int face_index,
  unsigned long long * winners)
{
  const float ax = width * 0.5f *
    (projection[0] * a.x / a.z + projection[2] + 1.0f);
  const float ay = height * 0.5f *
    (1.0f - (projection[1] * a.y / a.z + projection[3]));
  const float bx = width * 0.5f *
    (projection[0] * b.x / b.z + projection[2] + 1.0f);
  const float by = height * 0.5f *
    (1.0f - (projection[1] * b.y / b.z + projection[3]));
  const float cx = width * 0.5f *
    (projection[0] * c.x / c.z + projection[2] + 1.0f);
  const float cy = height * 0.5f *
    (1.0f - (projection[1] * c.y / c.z + projection[3]));
  const float area = edge(ax, ay, bx, by, cx, cy);
  if (fabsf(area) < 1e-8f) {
    return;
  }

  const int min_x = max(0, static_cast<int>(floorf(fminf(ax, fminf(bx, cx)))));
  const int max_x = min(width - 1, static_cast<int>(ceilf(fmaxf(ax, fmaxf(bx, cx)))));
  const int min_y = max(0, static_cast<int>(floorf(fminf(ay, fminf(by, cy)))));
  const int max_y = min(height - 1, static_cast<int>(ceilf(fmaxf(ay, fmaxf(by, cy)))));
  if (min_x > max_x || min_y > max_y) {
    return;
  }

  const float inverse_area = 1.0f / area;
  for (int y = min_y; y <= max_y; ++y) {
    const float sample_y = static_cast<float>(y) + 0.5f;
    for (int x = min_x; x <= max_x; ++x) {
      const float sample_x = static_cast<float>(x) + 0.5f;
      const float w0 = edge(bx, by, cx, cy, sample_x, sample_y) * inverse_area;
      const float w1 = edge(cx, cy, ax, ay, sample_x, sample_y) * inverse_area;
      const float w2 = 1.0f - w0 - w1;
      if (w0 < -1e-5f || w1 < -1e-5f || w2 < -1e-5f) {
        continue;
      }
      const float inverse_depth = w0 / a.z + w1 / b.z + w2 / c.z;
      if (inverse_depth <= 0.0f) {
        continue;
      }
      const float depth = 1.0f / inverse_depth;
      const unsigned int depth_bits = __float_as_uint(depth);
      const unsigned long long key =
        (static_cast<unsigned long long>(depth_bits) << 32) |
        static_cast<unsigned long long>(face_index);
      atomicMin(&winners[y * width + x], key);
    }
  }
}

__global__ void rasterize_triangles(
  const float3 * vertices,
  const uint3 * faces,
  std::size_t face_count,
  const float * camera_positions,
  const float * world_to_camera,
  const float * projections,
  int view_count,
  int width,
  int height,
  float near_m,
  float far_m,
  unsigned long long * winners)
{
  const std::size_t face_index =
    static_cast<std::size_t>(blockIdx.x) * blockDim.x + threadIdx.x;
  if (face_index >= face_count) {
    return;
  }

  const uint3 face = faces[face_index];
  const std::size_t pixel_count = static_cast<std::size_t>(width) * height;
  for (int view = 0; view < view_count; ++view) {
    const float * position = camera_positions + view * 3;
    const float * matrix = world_to_camera + view * 9;
    const float * projection = projections + view * 4;
    const float3 a = transform_vertex(vertices[face.x], position, matrix);
    const float3 b = transform_vertex(vertices[face.y], position, matrix);
    const float3 c = transform_vertex(vertices[face.z], position, matrix);
    float3 triangle[3] = {a, b, c};
    float3 near_clipped[6];
    float3 fully_clipped[8];
    const int near_count = clip_polygon_z(
      triangle,
      3,
      near_m,
      true,
      near_clipped);
    const int clipped_count = clip_polygon_z(
      near_clipped,
      near_count,
      far_m,
      false,
      fully_clipped);
    if (clipped_count < 3) {
      continue;
    }
    unsigned long long * view_winners = winners + view * pixel_count;
    for (int triangle_index = 1; triangle_index + 1 < clipped_count; ++triangle_index) {
      rasterize_projected_triangle(
        fully_clipped[0],
        fully_clipped[triangle_index],
        fully_clipped[triangle_index + 1],
        projection,
        width,
        height,
        static_cast<unsigned int>(face_index),
        view_winners);
    }
  }
}

__device__ uchar3 sample_texture_bilinear(
  const uchar3 * texture,
  int texture_width,
  int texture_height,
  const uint4 rectangle,
  float u,
  float v)
{
  u = u - floorf(u);
  v = v - floorf(v);
  const float sample_x = rectangle.x + u * max(1.0f, rectangle.z - 1.0f);
  const float sample_y = rectangle.y + v * max(1.0f, rectangle.w - 1.0f);
  const int x0 = max(0, min(texture_width - 1, static_cast<int>(floorf(sample_x))));
  const int y0 = max(0, min(texture_height - 1, static_cast<int>(floorf(sample_y))));
  const int x1 = min(
    static_cast<int>(rectangle.x + rectangle.z - 1),
    x0 + 1);
  const int y1 = min(
    static_cast<int>(rectangle.y + rectangle.w - 1),
    y0 + 1);
  const float tx = sample_x - floorf(sample_x);
  const float ty = sample_y - floorf(sample_y);
  const uchar3 c00 = texture[y0 * texture_width + x0];
  const uchar3 c10 = texture[y0 * texture_width + x1];
  const uchar3 c01 = texture[y1 * texture_width + x0];
  const uchar3 c11 = texture[y1 * texture_width + x1];
  const float3 top = make_float3(
    c00.x + (c10.x - c00.x) * tx,
    c00.y + (c10.y - c00.y) * tx,
    c00.z + (c10.z - c00.z) * tx);
  const float3 bottom = make_float3(
    c01.x + (c11.x - c01.x) * tx,
    c01.y + (c11.y - c01.y) * tx,
    c01.z + (c11.z - c01.z) * tx);
  return make_uchar3(
    static_cast<unsigned char>(top.x + (bottom.x - top.x) * ty + 0.5f),
    static_cast<unsigned char>(top.y + (bottom.y - top.y) * ty + 0.5f),
    static_cast<unsigned char>(top.z + (bottom.z - top.z) * ty + 0.5f));
}

__global__ void resolve_pixels(
  const unsigned long long * winners,
  const float3 * vertices,
  const uint3 * faces,
  const float2 * uvs,
  const uchar3 * tints,
  const uint4 * texture_rects,
  const uchar3 * texture_atlas,
  int texture_width,
  int texture_height,
  const float * camera_positions,
  const float * world_to_camera,
  const float * projections,
  int width,
  int height,
  std::size_t output_count,
  uchar3 * output_colors,
  float * output_depth)
{
  const std::size_t pixel =
    static_cast<std::size_t>(blockIdx.x) * blockDim.x + threadIdx.x;
  if (pixel >= output_count) {
    return;
  }
  const unsigned long long winner = winners[pixel];
  if (winner == 0xffffffffffffffffULL) {
    output_colors[pixel] = make_uchar3(0, 0, 0);
    output_depth[pixel] = __int_as_float(0x7f800000);
    return;
  }
  const unsigned int face_index = static_cast<unsigned int>(winner);
  const std::size_t pixels_per_view = static_cast<std::size_t>(width) * height;
  const int view = static_cast<int>(pixel / pixels_per_view);
  const std::size_t local_pixel = pixel - view * pixels_per_view;
  const int pixel_x = static_cast<int>(local_pixel % width);
  const int pixel_y = static_cast<int>(local_pixel / width);
  const float * position = camera_positions + view * 3;
  const float * matrix = world_to_camera + view * 9;
  const float * projection = projections + view * 4;
  const uint3 face = faces[face_index];
  const float3 a = transform_vertex(vertices[face.x], position, matrix);
  const float3 b = transform_vertex(vertices[face.y], position, matrix);
  const float3 c = transform_vertex(vertices[face.z], position, matrix);
  const float depth = __uint_as_float(static_cast<unsigned int>(winner >> 32));
  const float ndc_x =
    2.0f * (static_cast<float>(pixel_x) + 0.5f) / width - 1.0f;
  const float ndc_y =
    1.0f - 2.0f * (static_cast<float>(pixel_y) + 0.5f) / height;
  const float3 hit = make_float3(
    (ndc_x - projection[2]) * depth / projection[0],
    (ndc_y - projection[3]) * depth / projection[1],
    depth);
  const float3 side_ab = make_float3(b.x - a.x, b.y - a.y, b.z - a.z);
  const float3 side_ac = make_float3(c.x - a.x, c.y - a.y, c.z - a.z);
  const float3 side_hit = make_float3(
    hit.x - a.x,
    hit.y - a.y,
    hit.z - a.z);
  const float dot_ab_ab =
    side_ab.x * side_ab.x + side_ab.y * side_ab.y + side_ab.z * side_ab.z;
  const float dot_ab_ac =
    side_ab.x * side_ac.x + side_ab.y * side_ac.y + side_ab.z * side_ac.z;
  const float dot_ac_ac =
    side_ac.x * side_ac.x + side_ac.y * side_ac.y + side_ac.z * side_ac.z;
  const float dot_hit_ab =
    side_hit.x * side_ab.x + side_hit.y * side_ab.y + side_hit.z * side_ab.z;
  const float dot_hit_ac =
    side_hit.x * side_ac.x + side_hit.y * side_ac.y + side_hit.z * side_ac.z;
  const float denominator =
    dot_ab_ab * dot_ac_ac - dot_ab_ac * dot_ab_ac;
  float u = 0.0f;
  float v = 0.0f;
  if (fabsf(denominator) > 1e-12f) {
    const float weight_b =
      (dot_ac_ac * dot_hit_ab - dot_ab_ac * dot_hit_ac) / denominator;
    const float weight_c =
      (dot_ab_ab * dot_hit_ac - dot_ab_ac * dot_hit_ab) / denominator;
    const float weight_a = 1.0f - weight_b - weight_c;
    const float2 uv_a = uvs[face.x];
    const float2 uv_b = uvs[face.y];
    const float2 uv_c = uvs[face.z];
    u = weight_a * uv_a.x + weight_b * uv_b.x + weight_c * uv_c.x;
    v = weight_a * uv_a.y + weight_b * uv_b.y + weight_c * uv_c.y;
  }
  const uchar3 texture_color = sample_texture_bilinear(
    texture_atlas,
    texture_width,
    texture_height,
    texture_rects[face_index],
    u,
    v);
  const uchar3 tint = tints[face_index];
  output_colors[pixel] = make_uchar3(
    static_cast<unsigned char>(
      (static_cast<unsigned int>(texture_color.x) * tint.x + 127) / 255),
    static_cast<unsigned char>(
      (static_cast<unsigned int>(texture_color.y) * tint.y + 127) / 255),
    static_cast<unsigned char>(
      (static_cast<unsigned int>(texture_color.z) * tint.z + 127) / 255));
  output_depth[pixel] = depth;
}

bool ensure_output_capacity(Renderer * renderer, std::size_t output_count)
{
  if (renderer->output_capacity >= output_count) {
    return true;
  }
  cudaFree(renderer->winners);
  cudaFree(renderer->output_colors);
  cudaFree(renderer->output_depth);
  renderer->winners = nullptr;
  renderer->output_colors = nullptr;
  renderer->output_depth = nullptr;
  renderer->output_capacity = 0;
  if (!check_cuda(
      cudaMalloc(
        reinterpret_cast<void **>(&renderer->winners),
        output_count * sizeof(unsigned long long)),
      "cudaMalloc winners") ||
    !check_cuda(
      cudaMalloc(
        reinterpret_cast<void **>(&renderer->output_colors),
        output_count * sizeof(uchar3)),
      "cudaMalloc output colors") ||
    !check_cuda(
      cudaMalloc(
        reinterpret_cast<void **>(&renderer->output_depth),
        output_count * sizeof(float)),
      "cudaMalloc output depth")) {
    return false;
  }
  renderer->output_capacity = output_count;
  return true;
}

void release_renderer(Renderer * renderer)
{
  if (renderer == nullptr) {
    return;
  }
  cudaFree(renderer->vertices);
  cudaFree(renderer->faces);
  cudaFree(renderer->uvs);
  cudaFree(renderer->tints);
  cudaFree(renderer->texture_rects);
  cudaFree(renderer->texture_atlas);
  cudaFree(renderer->camera_positions);
  cudaFree(renderer->world_to_camera);
  cudaFree(renderer->projections);
  cudaFree(renderer->winners);
  cudaFree(renderer->output_colors);
  cudaFree(renderer->output_depth);
  if (renderer->begin != nullptr) cudaEventDestroy(renderer->begin);
  if (renderer->end != nullptr) cudaEventDestroy(renderer->end);
  delete renderer;
}
}  // namespace

extern "C"
{
const char * horus_cuda_mesh_last_error()
{
  return g_last_error.c_str();
}

int horus_cuda_mesh_create(
  std::size_t vertex_count,
  std::size_t face_count,
  const float * host_vertices,
  const unsigned int * host_faces,
  const float * host_uvs,
  const unsigned char * host_tints,
  const unsigned int * host_texture_rects,
  const unsigned char * host_texture_atlas,
  int texture_width,
  int texture_height,
  void ** output)
{
  g_last_error.clear();
  if (output == nullptr || host_vertices == nullptr || host_faces == nullptr ||
      host_uvs == nullptr || host_tints == nullptr ||
      host_texture_rects == nullptr || host_texture_atlas == nullptr ||
      vertex_count == 0 || face_count == 0 || face_count > 0xffffffffULL ||
      texture_width <= 0 || texture_height <= 0) {
    g_last_error = "invalid mesh input";
    return 1;
  }
  Renderer * renderer = new (std::nothrow) Renderer();
  if (renderer == nullptr) {
    g_last_error = "failed to allocate renderer state";
    return 2;
  }
  renderer->vertex_count = vertex_count;
  renderer->face_count = face_count;
  renderer->texture_width = texture_width;
  renderer->texture_height = texture_height;
  if (!check_cuda(
      cudaMalloc(
        reinterpret_cast<void **>(&renderer->vertices),
        vertex_count * sizeof(float3)),
      "cudaMalloc vertices") ||
    !check_cuda(
      cudaMalloc(
        reinterpret_cast<void **>(&renderer->faces),
        face_count * sizeof(uint3)),
      "cudaMalloc faces") ||
    !check_cuda(
      cudaMalloc(
        reinterpret_cast<void **>(&renderer->uvs),
        vertex_count * sizeof(float2)),
      "cudaMalloc uvs") ||
    !check_cuda(
      cudaMalloc(
        reinterpret_cast<void **>(&renderer->tints),
        face_count * sizeof(uchar3)),
      "cudaMalloc tints") ||
    !check_cuda(
      cudaMalloc(
        reinterpret_cast<void **>(&renderer->texture_rects),
        face_count * sizeof(uint4)),
      "cudaMalloc texture rectangles") ||
    !check_cuda(
      cudaMalloc(
        reinterpret_cast<void **>(&renderer->texture_atlas),
        static_cast<std::size_t>(texture_width) * texture_height * sizeof(uchar3)),
      "cudaMalloc texture atlas") ||
    !check_cuda(
      cudaMalloc(
        reinterpret_cast<void **>(&renderer->camera_positions),
        kMaximumViews * 3 * sizeof(float)),
      "cudaMalloc camera positions") ||
    !check_cuda(
      cudaMalloc(
        reinterpret_cast<void **>(&renderer->world_to_camera),
        kMaximumViews * 9 * sizeof(float)),
      "cudaMalloc camera matrices") ||
    !check_cuda(
      cudaMalloc(
        reinterpret_cast<void **>(&renderer->projections),
        kMaximumViews * 4 * sizeof(float)),
      "cudaMalloc projections") ||
    !check_cuda(cudaEventCreate(&renderer->begin), "cudaEventCreate begin") ||
    !check_cuda(cudaEventCreate(&renderer->end), "cudaEventCreate end") ||
    !check_cuda(
      cudaMemcpy(
        renderer->vertices,
        host_vertices,
        vertex_count * sizeof(float3),
        cudaMemcpyHostToDevice),
      "cudaMemcpy vertices") ||
    !check_cuda(
      cudaMemcpy(
        renderer->faces,
        host_faces,
        face_count * sizeof(uint3),
        cudaMemcpyHostToDevice),
      "cudaMemcpy faces") ||
    !check_cuda(
      cudaMemcpy(
        renderer->uvs,
        host_uvs,
        vertex_count * sizeof(float2),
        cudaMemcpyHostToDevice),
      "cudaMemcpy uvs") ||
    !check_cuda(
      cudaMemcpy(
        renderer->tints,
        host_tints,
        face_count * sizeof(uchar3),
        cudaMemcpyHostToDevice),
      "cudaMemcpy tints") ||
    !check_cuda(
      cudaMemcpy(
        renderer->texture_rects,
        host_texture_rects,
        face_count * sizeof(uint4),
        cudaMemcpyHostToDevice),
      "cudaMemcpy texture rectangles") ||
    !check_cuda(
      cudaMemcpy(
        renderer->texture_atlas,
        host_texture_atlas,
        static_cast<std::size_t>(texture_width) * texture_height * sizeof(uchar3),
        cudaMemcpyHostToDevice),
      "cudaMemcpy texture atlas")) {
    release_renderer(renderer);
    return 3;
  }
  *output = renderer;
  return 0;
}

int horus_cuda_mesh_render_views(
  void * handle,
  const float * camera_positions,
  const float * world_to_camera,
  const float * projections,
  int view_count,
  int width,
  int height,
  float vertical_fov_degrees,
  float near_m,
  float far_m,
  unsigned char * host_color,
  float * host_depth,
  float * elapsed_ms)
{
  g_last_error.clear();
  Renderer * renderer = static_cast<Renderer *>(handle);
  if (renderer == nullptr || camera_positions == nullptr ||
      world_to_camera == nullptr || host_color == nullptr ||
      projections == nullptr || host_depth == nullptr ||
      view_count <= 0 || view_count > kMaximumViews ||
      width <= 0 || height <= 0 || near_m <= 0.0f || far_m <= near_m ||
      vertical_fov_degrees <= 1.0f || vertical_fov_degrees >= 179.0f) {
    g_last_error = "invalid render arguments";
    return 1;
  }

  const std::size_t output_count =
    static_cast<std::size_t>(view_count) * width * height;
  if (!ensure_output_capacity(renderer, output_count)) {
    return 2;
  }
  if (!check_cuda(
      cudaMemcpy(
        renderer->camera_positions,
        camera_positions,
        view_count * 3 * sizeof(float),
        cudaMemcpyHostToDevice),
      "cudaMemcpy camera positions") ||
    !check_cuda(
      cudaMemcpy(
        renderer->world_to_camera,
        world_to_camera,
        view_count * 9 * sizeof(float),
        cudaMemcpyHostToDevice),
      "cudaMemcpy camera matrices") ||
    !check_cuda(
      cudaMemcpy(
        renderer->projections,
        projections,
        view_count * 4 * sizeof(float),
        cudaMemcpyHostToDevice),
      "cudaMemcpy projections") ||
    !check_cuda(
      cudaMemset(
        renderer->winners,
        0xff,
        output_count * sizeof(unsigned long long)),
      "cudaMemset winners")) {
    return 3;
  }

  cudaEventRecord(renderer->begin);
  const int threads = 128;
  const int face_blocks =
    static_cast<int>((renderer->face_count + threads - 1) / threads);
  rasterize_triangles<<<face_blocks, threads>>>(
    renderer->vertices,
    renderer->faces,
    renderer->face_count,
    renderer->camera_positions,
    renderer->world_to_camera,
    renderer->projections,
    view_count,
    width,
    height,
    near_m,
    far_m,
    renderer->winners);
  const int pixel_threads = 256;
  const int pixel_blocks =
    static_cast<int>((output_count + pixel_threads - 1) / pixel_threads);
  resolve_pixels<<<pixel_blocks, pixel_threads>>>(
    renderer->winners,
    renderer->vertices,
    renderer->faces,
    renderer->uvs,
    renderer->tints,
    renderer->texture_rects,
    renderer->texture_atlas,
    renderer->texture_width,
    renderer->texture_height,
    renderer->camera_positions,
    renderer->world_to_camera,
    renderer->projections,
    width,
    height,
    output_count,
    renderer->output_colors,
    renderer->output_depth);
  cudaEventRecord(renderer->end);
  if (!check_cuda(cudaGetLastError(), "CUDA mesh render kernel") ||
      !check_cuda(cudaEventSynchronize(renderer->end), "CUDA mesh synchronize") ||
      !check_cuda(
        cudaMemcpy(
          host_color,
          renderer->output_colors,
          output_count * sizeof(uchar3),
          cudaMemcpyDeviceToHost),
        "cudaMemcpy output color") ||
      !check_cuda(
        cudaMemcpy(
          host_depth,
          renderer->output_depth,
          output_count * sizeof(float),
          cudaMemcpyDeviceToHost),
        "cudaMemcpy output depth")) {
    return 4;
  }
  if (elapsed_ms != nullptr) {
    cudaEventElapsedTime(elapsed_ms, renderer->begin, renderer->end);
  }
  return 0;
}

void horus_cuda_mesh_destroy(void * handle)
{
  release_renderer(static_cast<Renderer *>(handle));
}
}
