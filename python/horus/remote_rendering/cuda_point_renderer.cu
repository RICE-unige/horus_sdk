#include <cuda_runtime.h>

#include <cstdint>
#include <new>
#include <string>

namespace
{
constexpr int kMaximumViews = 12;
thread_local std::string g_last_error;

struct Renderer
{
  std::size_t point_count = 0;
  float * points = nullptr;
  uchar3 * colors = nullptr;
  float * camera_positions = nullptr;
  float * world_to_camera = nullptr;
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

__global__ void project_points_multiview(
  const float * points,
  std::size_t point_count,
  const float * camera_positions,
  const float * world_to_camera,
  int view_count,
  int width,
  int height,
  float focal,
  float near_m,
  float far_m,
  int point_radius,
  unsigned long long * winners)
{
  const std::size_t index = static_cast<std::size_t>(blockIdx.x) * blockDim.x + threadIdx.x;
  if (index >= point_count) {
    return;
  }

  const float point_x = points[index * 3 + 0];
  const float point_y = points[index * 3 + 1];
  const float point_z = points[index * 3 + 2];
  const std::size_t pixel_count = static_cast<std::size_t>(width) * height;
  for (int view = 0; view < view_count; ++view) {
    const float * position = camera_positions + view * 3;
    const float * matrix = world_to_camera + view * 9;
    const float px = point_x - position[0];
    const float py = point_y - position[1];
    const float pz = point_z - position[2];
    const float camera_x = matrix[0] * px + matrix[1] * py + matrix[2] * pz;
    const float camera_y = matrix[3] * px + matrix[4] * py + matrix[5] * pz;
    const float camera_z = matrix[6] * px + matrix[7] * py + matrix[8] * pz;
    if (!(camera_z >= near_m && camera_z <= far_m)) {
      continue;
    }

    const int screen_x = __float2int_rd(width * 0.5f + focal * camera_x / camera_z);
    const int screen_y = __float2int_rd(height * 0.5f - focal * camera_y / camera_z);
    if (screen_x < -point_radius || screen_x >= width + point_radius ||
        screen_y < -point_radius || screen_y >= height + point_radius) {
      continue;
    }

    const unsigned int depth_bits = __float_as_uint(camera_z);
    const unsigned long long key =
      (static_cast<unsigned long long>(depth_bits) << 32) |
      static_cast<unsigned long long>(static_cast<unsigned int>(index));
    unsigned long long * view_winners = winners + view * pixel_count;
    for (int dy = -point_radius; dy <= point_radius; ++dy) {
      for (int dx = -point_radius; dx <= point_radius; ++dx) {
        if (dx * dx + dy * dy > point_radius * point_radius + 1) {
          continue;
        }
        const int target_x = screen_x + dx;
        const int target_y = screen_y + dy;
        if (target_x < 0 || target_x >= width || target_y < 0 || target_y >= height) {
          continue;
        }
        atomicMin(&view_winners[target_y * width + target_x], key);
      }
    }
  }
}

__global__ void resolve_pixels(
  const unsigned long long * winners,
  const uchar3 * colors,
  std::size_t output_count,
  uchar3 * output_colors,
  float * output_depth)
{
  const std::size_t pixel = static_cast<std::size_t>(blockIdx.x) * blockDim.x + threadIdx.x;
  if (pixel >= output_count) {
    return;
  }
  const unsigned long long winner = winners[pixel];
  if (winner == 0xffffffffffffffffULL) {
    output_colors[pixel] = make_uchar3(0, 0, 0);
    output_depth[pixel] = __int_as_float(0x7f800000);
    return;
  }
  const unsigned int point_index = static_cast<unsigned int>(winner & 0xffffffffULL);
  const unsigned int depth_bits = static_cast<unsigned int>(winner >> 32);
  output_colors[pixel] = colors[point_index];
  output_depth[pixel] = __uint_as_float(depth_bits);
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
      cudaMalloc(reinterpret_cast<void **>(&renderer->winners), output_count * sizeof(unsigned long long)),
      "cudaMalloc winners") ||
    !check_cuda(
      cudaMalloc(reinterpret_cast<void **>(&renderer->output_colors), output_count * sizeof(uchar3)),
      "cudaMalloc output colors") ||
    !check_cuda(
      cudaMalloc(reinterpret_cast<void **>(&renderer->output_depth), output_count * sizeof(float)),
      "cudaMalloc output depth")) {
    return false;
  }
  renderer->output_capacity = output_count;
  return true;
}

int render_views_impl(
  Renderer * renderer,
  const float * camera_positions,
  const float * world_to_camera,
  int view_count,
  int width,
  int height,
  float vertical_fov_degrees,
  float near_m,
  float far_m,
  int point_radius,
  unsigned char * host_color,
  float * host_depth,
  float * elapsed_ms)
{
  if (renderer == nullptr || camera_positions == nullptr || world_to_camera == nullptr ||
      host_color == nullptr || host_depth == nullptr || view_count <= 0 ||
      view_count > kMaximumViews || width <= 0 || height <= 0 || near_m <= 0.0f ||
      far_m <= near_m || vertical_fov_degrees <= 1.0f || vertical_fov_degrees >= 179.0f ||
      point_radius < 0 || point_radius > 4) {
    g_last_error = "invalid render arguments";
    return 1;
  }
  const std::size_t pixel_count = static_cast<std::size_t>(width) * height;
  const std::size_t output_count = pixel_count * view_count;
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
      cudaMemset(renderer->winners, 0xff, output_count * sizeof(unsigned long long)),
      "cudaMemset winners")) {
    return 3;
  }

  cudaEventRecord(renderer->begin);
  const float radians = vertical_fov_degrees * 0.01745329251994329577f;
  const float focal = height * 0.5f / tanf(radians * 0.5f);
  const int threads = 256;
  const int point_blocks = static_cast<int>((renderer->point_count + threads - 1) / threads);
  project_points_multiview<<<point_blocks, threads>>>(
    renderer->points,
    renderer->point_count,
    renderer->camera_positions,
    renderer->world_to_camera,
    view_count,
    width,
    height,
    focal,
    near_m,
    far_m,
    point_radius,
    renderer->winners);
  const int pixel_blocks = static_cast<int>((output_count + threads - 1) / threads);
  resolve_pixels<<<pixel_blocks, threads>>>(
    renderer->winners,
    renderer->colors,
    output_count,
    renderer->output_colors,
    renderer->output_depth);
  cudaEventRecord(renderer->end);
  if (!check_cuda(cudaGetLastError(), "CUDA render kernel") ||
      !check_cuda(cudaEventSynchronize(renderer->end), "CUDA render synchronize") ||
      !check_cuda(
        cudaMemcpy(host_color, renderer->output_colors, output_count * sizeof(uchar3), cudaMemcpyDeviceToHost),
        "cudaMemcpy output color") ||
      !check_cuda(
        cudaMemcpy(host_depth, renderer->output_depth, output_count * sizeof(float), cudaMemcpyDeviceToHost),
        "cudaMemcpy output depth")) {
    return 4;
  }
  if (elapsed_ms != nullptr) {
    cudaEventElapsedTime(elapsed_ms, renderer->begin, renderer->end);
  }
  return 0;
}
}  // namespace

extern "C"
{
const char * horus_cuda_last_error()
{
  return g_last_error.c_str();
}

int horus_cuda_create(std::size_t point_count, void ** output)
{
  g_last_error.clear();
  if (output == nullptr || point_count == 0 || point_count > 0xffffffffULL) {
    g_last_error = "invalid point count or output pointer";
    return 1;
  }
  Renderer * renderer = new (std::nothrow) Renderer();
  if (renderer == nullptr) {
    g_last_error = "failed to allocate renderer state";
    return 2;
  }
  renderer->point_count = point_count;
  if (!check_cuda(
      cudaMalloc(reinterpret_cast<void **>(&renderer->points), point_count * 3 * sizeof(float)),
      "cudaMalloc points") ||
    !check_cuda(
      cudaMalloc(reinterpret_cast<void **>(&renderer->colors), point_count * sizeof(uchar3)),
      "cudaMalloc colors") ||
    !check_cuda(
      cudaMalloc(reinterpret_cast<void **>(&renderer->camera_positions), kMaximumViews * 3 * sizeof(float)),
      "cudaMalloc camera positions") ||
    !check_cuda(
      cudaMalloc(reinterpret_cast<void **>(&renderer->world_to_camera), kMaximumViews * 9 * sizeof(float)),
      "cudaMalloc camera matrices") ||
    !check_cuda(cudaEventCreate(&renderer->begin), "cudaEventCreate begin") ||
    !check_cuda(cudaEventCreate(&renderer->end), "cudaEventCreate end")) {
    cudaFree(renderer->points);
    cudaFree(renderer->colors);
    cudaFree(renderer->camera_positions);
    cudaFree(renderer->world_to_camera);
    if (renderer->begin != nullptr) cudaEventDestroy(renderer->begin);
    if (renderer->end != nullptr) cudaEventDestroy(renderer->end);
    delete renderer;
    return 3;
  }
  *output = renderer;
  return 0;
}

int horus_cuda_upload(
  void * handle,
  std::size_t offset,
  std::size_t count,
  const float * points,
  const unsigned char * colors)
{
  g_last_error.clear();
  Renderer * renderer = static_cast<Renderer *>(handle);
  if (renderer == nullptr || points == nullptr || colors == nullptr ||
      offset > renderer->point_count || count > renderer->point_count - offset) {
    g_last_error = "invalid upload range or pointer";
    return 1;
  }
  if (!check_cuda(
      cudaMemcpy(
        renderer->points + offset * 3,
        points,
        count * 3 * sizeof(float),
        cudaMemcpyHostToDevice),
      "cudaMemcpy points") ||
    !check_cuda(
      cudaMemcpy(
        renderer->colors + offset,
        colors,
        count * sizeof(uchar3),
        cudaMemcpyHostToDevice),
      "cudaMemcpy colors")) {
    return 2;
  }
  return 0;
}

int horus_cuda_render_views(
  void * handle,
  const float * camera_positions,
  const float * world_to_camera,
  int view_count,
  int width,
  int height,
  float vertical_fov_degrees,
  float near_m,
  float far_m,
  int point_radius,
  unsigned char * host_color,
  float * host_depth,
  float * elapsed_ms)
{
  g_last_error.clear();
  return render_views_impl(
    static_cast<Renderer *>(handle),
    camera_positions,
    world_to_camera,
    view_count,
    width,
    height,
    vertical_fov_degrees,
    near_m,
    far_m,
    point_radius,
    host_color,
    host_depth,
    elapsed_ms);
}

int horus_cuda_render(
  void * handle,
  const float * camera_position,
  const float * world_to_camera,
  int width,
  int height,
  float vertical_fov_degrees,
  float near_m,
  float far_m,
  int point_radius,
  unsigned char * host_color,
  float * host_depth,
  float * elapsed_ms)
{
  g_last_error.clear();
  return render_views_impl(
    static_cast<Renderer *>(handle),
    camera_position,
    world_to_camera,
    1,
    width,
    height,
    vertical_fov_degrees,
    near_m,
    far_m,
    point_radius,
    host_color,
    host_depth,
    elapsed_ms);
}

int horus_pack_views(
  const unsigned char * view_colors,
  const float * view_depth,
  int view_count,
  int tile_width,
  int tile_height,
  float near_m,
  float far_m,
  unsigned char * packed_rgb)
{
  g_last_error.clear();
  if (view_colors == nullptr || view_depth == nullptr || packed_rgb == nullptr ||
      view_count != 4 || tile_width <= 0 || tile_height <= 0 || tile_width % 2 != 0 ||
      near_m <= 0.0f || far_m <= near_m) {
    g_last_error = "invalid packed-atlas arguments";
    return 1;
  }
  const int atlas_width = tile_width * 2;
  const int atlas_height = tile_height * 2;
  const int packed_width = atlas_width * 2;
  const std::size_t tile_pixels = static_cast<std::size_t>(tile_width) * tile_height;
  const float depth_scale = 255.0f / (far_m - near_m);
  constexpr int luma_min = 32;
  constexpr int luma_span = 223;

  for (int y = 0; y < atlas_height; ++y) {
    const int view_row = y < tile_height ? 1 : 0;
    const int local_y = y % tile_height;
    for (int x = 0; x < atlas_width; ++x) {
      const int view_column = x < tile_width ? 0 : 1;
      const int view = view_row * 2 + view_column;
      const int local_x = x % tile_width;
      const std::size_t source = static_cast<std::size_t>(view) * tile_pixels +
        static_cast<std::size_t>(local_y) * tile_width + local_x;
      const std::size_t target =
        (static_cast<std::size_t>(y) * packed_width + x) * 3;
      const std::size_t color_source = source * 3;
      packed_rgb[target + 0] = view_colors[color_source + 0];
      packed_rgb[target + 1] = view_colors[color_source + 1];
      packed_rgb[target + 2] = view_colors[color_source + 2];
    }

    for (int x = 0; x < atlas_width; ++x) {
      const int sampled_atlas_x = (x / 2) * 2;
      const int view_column = sampled_atlas_x < tile_width ? 0 : 1;
      const int view = view_row * 2 + view_column;
      const int local_x = sampled_atlas_x % tile_width;
      const std::size_t source = static_cast<std::size_t>(view) * tile_pixels +
        static_cast<std::size_t>(local_y) * tile_width + local_x;
      const float depth = view_depth[source];
      unsigned char luma = 0;
      if (depth >= near_m && depth <= far_m) {
        const float normalized = fminf(fmaxf((depth - near_m) * depth_scale, 0.0f), 255.0f);
        const unsigned int code = static_cast<unsigned int>(normalized + 0.5f);
        const unsigned int nibble = x % 2 == 0 ? code >> 4 : code & 0x0f;
        luma = static_cast<unsigned char>(
          luma_min + static_cast<int>(nibble * luma_span / 15.0f + 0.5f));
      }
      const std::size_t target =
        (static_cast<std::size_t>(y) * packed_width + atlas_width + x) * 3;
      packed_rgb[target + 0] = luma;
      packed_rgb[target + 1] = luma;
      packed_rgb[target + 2] = luma;
    }
  }
  return 0;
}

void horus_cuda_destroy(void * handle)
{
  Renderer * renderer = static_cast<Renderer *>(handle);
  if (renderer == nullptr) {
    return;
  }
  cudaFree(renderer->points);
  cudaFree(renderer->colors);
  cudaFree(renderer->camera_positions);
  cudaFree(renderer->world_to_camera);
  cudaFree(renderer->winners);
  cudaFree(renderer->output_colors);
  cudaFree(renderer->output_depth);
  if (renderer->begin != nullptr) cudaEventDestroy(renderer->begin);
  if (renderer->end != nullptr) cudaEventDestroy(renderer->end);
  delete renderer;
}
}
