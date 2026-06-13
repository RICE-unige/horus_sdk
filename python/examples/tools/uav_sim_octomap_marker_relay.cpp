#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace
{
using OctomapMsg = octomap_msgs::msg::Octomap;
using MarkerMsg = visualization_msgs::msg::Marker;
using PointMsg = geometry_msgs::msg::Point;

enum class MarkerStyle
{
  RvizVoxels,
  SurfaceMesh,
};

MarkerStyle parse_marker_style(const std::string & value)
{
  return value == "surface_mesh" ? MarkerStyle::SurfaceMesh : MarkerStyle::RvizVoxels;
}

const char * marker_style_name(MarkerStyle style)
{
  return style == MarkerStyle::SurfaceMesh ? "surface_mesh" : "rviz_voxels";
}

PointMsg point(double x, double y, double z)
{
  PointMsg p;
  p.x = x;
  p.y = y;
  p.z = z;
  return p;
}

void add_triangle(MarkerMsg & marker, const PointMsg & a, const PointMsg & b, const PointMsg & c)
{
  marker.points.push_back(a);
  marker.points.push_back(b);
  marker.points.push_back(c);
}

void add_face(
  MarkerMsg & marker,
  const PointMsg & a,
  const PointMsg & b,
  const PointMsg & c,
  const PointMsg & d)
{
  add_triangle(marker, a, b, c);
  add_triangle(marker, a, c, d);
}

bool is_occupied_at(const octomap::OcTree & tree, double x, double y, double z)
{
  const auto * node = tree.search(x, y, z);
  return node != nullptr && tree.isNodeOccupied(node);
}

std::int64_t quantize_coordinate(double value, double unit)
{
  return static_cast<std::int64_t>(std::llround(value / unit));
}

struct CellKey
{
  std::int64_t u = 0;
  std::int64_t v = 0;

  bool operator==(const CellKey & other) const
  {
    return u == other.u && v == other.v;
  }

  bool operator<(const CellKey & other) const
  {
    return std::tie(v, u) < std::tie(other.v, other.u);
  }
};

struct CellKeyHash
{
  std::size_t operator()(const CellKey & key) const
  {
    const std::size_t h1 = std::hash<std::int64_t>{}(key.u);
    const std::size_t h2 = std::hash<std::int64_t>{}(key.v);
    return h1 ^ (h2 + 0x9e3779b97f4a7c15ULL + (h1 << 6) + (h1 >> 2));
  }
};

struct PlaneKey
{
  int axis = 0;
  int side = 1;
  std::int64_t plane = 0;
  std::int64_t cell_size = 1;

  bool operator==(const PlaneKey & other) const
  {
    return axis == other.axis &&
      side == other.side &&
      plane == other.plane &&
      cell_size == other.cell_size;
  }

  bool operator<(const PlaneKey & other) const
  {
    return std::tie(axis, side, plane, cell_size) <
      std::tie(other.axis, other.side, other.plane, other.cell_size);
  }
};

struct PlaneKeyHash
{
  std::size_t operator()(const PlaneKey & key) const
  {
    std::size_t h = std::hash<int>{}(key.axis);
    h ^= std::hash<int>{}(key.side) + 0x9e3779b97f4a7c15ULL + (h << 6) + (h >> 2);
    h ^= std::hash<std::int64_t>{}(key.plane) + 0x9e3779b97f4a7c15ULL + (h << 6) + (h >> 2);
    h ^= std::hash<std::int64_t>{}(key.cell_size) + 0x9e3779b97f4a7c15ULL + (h << 6) + (h >> 2);
    return h;
  }
};

struct FaceCell
{
  double plane = 0.0;
  double u_min = 0.0;
  double u_max = 0.0;
  double v_min = 0.0;
  double v_max = 0.0;
};

using FaceCellMap = std::unordered_map<CellKey, FaceCell, CellKeyHash>;
using FacePlaneMap = std::unordered_map<PlaneKey, FaceCellMap, PlaneKeyHash>;

void add_merged_face(
  MarkerMsg & marker,
  int axis,
  int side,
  double plane,
  double u0,
  double u1,
  double v0,
  double v1)
{
  PointMsg p00;
  PointMsg p10;
  PointMsg p11;
  PointMsg p01;

  if (axis == 0)
  {
    p00 = point(plane, u0, v0);
    p10 = point(plane, u1, v0);
    p11 = point(plane, u1, v1);
    p01 = point(plane, u0, v1);
  }
  else if (axis == 1)
  {
    p00 = point(u0, plane, v0);
    p10 = point(u1, plane, v0);
    p11 = point(u1, plane, v1);
    p01 = point(u0, plane, v1);
  }
  else
  {
    p00 = point(u0, v0, plane);
    p10 = point(u1, v0, plane);
    p11 = point(u1, v1, plane);
    p01 = point(u0, v1, plane);
  }

  bool use_positive_winding = side > 0;
  if (axis == 1)
  {
    use_positive_winding = !use_positive_winding;
  }

  if (use_positive_winding)
  {
    add_face(marker, p00, p10, p11, p01);
  }
  else
  {
    add_face(marker, p00, p01, p11, p10);
  }
}
}  // namespace

class OctomapMarkerRelay final : public rclcpp::Node
{
public:
  OctomapMarkerRelay(
    const std::string & input_topic,
    const std::string & output_topic,
    std::size_t max_triangles,
    double alpha,
    double voxel_scale,
    const std::string & marker_style,
    double min_update_interval_sec,
    int chunk_publish_period_ms,
    std::size_t chunks_per_tick)
  : Node("uav_sim_octomap_marker_relay"),
    input_topic_(input_topic),
    output_topic_(output_topic),
    max_triangles_(std::max<std::size_t>(12, max_triangles)),
    alpha_(std::clamp(alpha, 0.05, 1.0)),
    voxel_scale_(std::clamp(voxel_scale, 0.25, 8.0)),
    marker_style_(parse_marker_style(marker_style)),
    min_update_interval_(std::chrono::duration<double>(std::max(0.0, min_update_interval_sec))),
    chunk_publish_period_(std::chrono::milliseconds(std::max(1, chunk_publish_period_ms))),
    chunks_per_tick_(std::max<std::size_t>(1, chunks_per_tick))
  {
    auto input_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    auto output_qos = rclcpp::QoS(rclcpp::KeepLast(8)).reliable();
    publisher_ = create_publisher<MarkerMsg>(output_topic_, output_qos);
    subscription_ = create_subscription<OctomapMsg>(
      input_topic_,
      input_qos,
      [this](const OctomapMsg::SharedPtr msg) { handle_octomap(msg); });
    publish_timer_ = create_wall_timer(
      chunk_publish_period_,
      [this]() { on_publish_timer(); });

    RCLCPP_INFO(
      get_logger(),
      "Relaying %s octomap_msgs/Octomap to %s visualization_msgs/Marker TRIANGLE_LIST "
      "style=%s max_triangles=%zu voxel_scale=%.2f min_update_interval=%.2fs "
      "chunk_period=%ldms chunks_per_tick=%zu",
      input_topic_.c_str(),
      output_topic_.c_str(),
      marker_style_name(marker_style_),
      max_triangles_,
      voxel_scale_,
      min_update_interval_sec,
      chunk_publish_period_.count(),
      chunks_per_tick_);
  }

private:
  static std::uint64_t compute_source_signature(const OctomapMsg & msg)
  {
    std::uint64_t hash = 1469598103934665603ULL;
    auto mix = [&](std::uint64_t value) {
        hash ^= value;
        hash *= 1099511628211ULL;
      };

    mix(static_cast<std::uint64_t>(msg.binary));
    mix(static_cast<std::uint64_t>(std::llround(msg.resolution * 1000000.0)));
    mix(static_cast<std::uint64_t>(msg.data.size()));
    for (char c : msg.id)
    {
      mix(static_cast<unsigned char>(c));
    }

    if (!msg.data.empty())
    {
      const std::size_t stride = std::max<std::size_t>(1, msg.data.size() / 4096);
      for (std::size_t i = 0; i < msg.data.size(); i += stride)
      {
        mix(static_cast<unsigned char>(msg.data[i]));
      }
      mix(static_cast<unsigned char>(msg.data.back()));
    }

    return hash;
  }

  void handle_octomap(const OctomapMsg::SharedPtr msg)
  {
    if (!msg)
    {
      return;
    }

    latest_msg_ = msg;
    latest_signature_ = compute_source_signature(*msg);
    pending_rebuild_ = true;
    try_build_latest(false);
  }

  void try_build_latest(bool force)
  {
    if (!latest_msg_)
    {
      return;
    }

    const std::size_t subscribers = publisher_->get_subscription_count();
    if (subscribers == 0)
    {
      return;
    }

    if (!force && latest_signature_ == last_built_signature_)
    {
      pending_rebuild_ = false;
      return;
    }

    if (!force && !pending_markers_.empty())
    {
      return;
    }

    const auto now_steady = std::chrono::steady_clock::now();
    if (!force && has_last_build_time_ && (now_steady - last_build_time_) < min_update_interval_)
    {
      return;
    }

    build_and_queue_octomap(*latest_msg_, latest_signature_);
    pending_rebuild_ = false;
    last_built_signature_ = latest_signature_;
    last_build_time_ = now_steady;
    has_last_build_time_ = true;
  }

  void build_and_queue_octomap(const OctomapMsg & msg, std::uint64_t source_signature)
  {
    std::unique_ptr<octomap::AbstractOcTree> abstract_tree(octomap_msgs::msgToMap(msg));
    if (!abstract_tree)
    {
      RCLCPP_WARN(get_logger(), "Failed to deserialize Octomap message id=%s", msg.id.c_str());
      return;
    }

    const auto * tree = dynamic_cast<const octomap::OcTree *>(abstract_tree.get());
    if (tree == nullptr)
    {
      RCLCPP_WARN(
        get_logger(),
        "Unsupported Octomap tree type '%s'; only OcTree is currently converted",
        abstract_tree->getTreeType().c_str());
      return;
    }

    std::size_t occupied_count = 0;
    for (auto it = tree->begin_leafs(), end = tree->end_leafs(); it != end; ++it)
    {
      if (tree->isNodeOccupied(*it))
      {
        ++occupied_count;
      }
    }

    if (occupied_count == 0)
    {
      RCLCPP_WARN(get_logger(), "Octomap contained no occupied leaf nodes");
      return;
    }

    pending_markers_.clear();
    if (marker_style_ == MarkerStyle::RvizVoxels)
    {
      publish_full_voxel_markers(msg, *tree, occupied_count);
    }
    else
    {
      publish_surface_mesh_markers(msg, *tree, occupied_count);
    }

    RCLCPP_INFO(
      get_logger(),
      "Queued octomap marker update: signature=%zu pending_chunks=%zu subscribers=%zu",
      static_cast<std::size_t>(source_signature),
      pending_markers_.size(),
      publisher_->get_subscription_count());
  }

  void on_publish_timer()
  {
    const std::size_t subscribers = publisher_->get_subscription_count();
    if (last_subscription_count_ == 0 && subscribers > 0)
    {
      pending_rebuild_ = true;
      try_build_latest(true);
    }
    last_subscription_count_ = subscribers;

    if (subscribers == 0)
    {
      return;
    }

    std::size_t published = 0;
    while (!pending_markers_.empty() && published < chunks_per_tick_)
    {
      publisher_->publish(pending_markers_.front());
      pending_markers_.pop_front();
      ++published;
    }

    if (pending_markers_.empty() && pending_rebuild_)
    {
      try_build_latest(false);
    }
  }

  MarkerMsg make_base_marker(const OctomapMsg & msg, const char * ns, int id, int action) const
  {
    MarkerMsg marker;
    marker.header = msg.header;
    if (marker.header.frame_id.empty())
    {
      marker.header.frame_id = "map";
    }
    marker.header.stamp = now();
    marker.ns = ns;
    marker.id = id;
    marker.type = MarkerMsg::TRIANGLE_LIST;
    marker.action = action;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
    marker.color.r = 0.62f;
    marker.color.g = 0.72f;
    marker.color.b = 0.86f;
    marker.color.a = static_cast<float>(alpha_);
    return marker;
  }

  void publish_initial_clear_once(const OctomapMsg & msg, const char * ns)
  {
    if (sent_initial_clear_)
    {
      return;
    }

    queue_marker(make_base_marker(msg, ns, 0, MarkerMsg::DELETEALL));
    sent_initial_clear_ = true;
    last_published_chunks_ = 0;
  }

  void queue_marker(const MarkerMsg & marker)
  {
    pending_markers_.push_back(marker);
  }

  void publish_surface_mesh_markers(
    const OctomapMsg & msg,
    const octomap::OcTree & tree,
    std::size_t occupied_count)
  {
    constexpr const char * ns = "uav_sim_octomap_surface_chunked";
    publish_initial_clear_once(msg, ns);

    const double resolution = tree.getResolution();
    FacePlaneMap planes;
    planes.reserve(occupied_count);
    std::size_t visited_voxels = 0;
    std::size_t exposed_cell_faces = 0;

    auto collect_face = [&](
        int axis,
        int side,
        bool neighbor_occupied,
        double cx,
        double cy,
        double cz,
        double size) {
        if (neighbor_occupied)
        {
          return;
        }

        const double visual_half = size * voxel_scale_ * 0.5;
        const std::int64_t cell_size = std::max<std::int64_t>(
          1,
          quantize_coordinate(size, resolution));
        double center[3] = {cx, cy, cz};
        const int u_axis = axis == 0 ? 1 : 0;
        const int v_axis = axis == 2 ? 1 : 2;
        const double logical_plane = center[axis] + static_cast<double>(side) * size * 0.5;
        const double visual_plane = center[axis] + static_cast<double>(side) * visual_half;
        PlaneKey plane_key{
          axis,
          side,
          quantize_coordinate(logical_plane, resolution),
          cell_size};
        CellKey cell_key{
          quantize_coordinate(center[u_axis] - size * 0.5, size),
          quantize_coordinate(center[v_axis] - size * 0.5, size)};

        FaceCell cell;
        cell.plane = visual_plane;
        cell.u_min = center[u_axis] - visual_half;
        cell.u_max = center[u_axis] + visual_half;
        cell.v_min = center[v_axis] - visual_half;
        cell.v_max = center[v_axis] + visual_half;
        planes[plane_key][cell_key] = cell;
        ++exposed_cell_faces;
      };

    for (auto it = tree.begin_leafs(), end = tree.end_leafs(); it != end; ++it)
    {
      if (!tree.isNodeOccupied(*it))
      {
        continue;
      }

      ++visited_voxels;
      const double cx = it.getX();
      const double cy = it.getY();
      const double cz = it.getZ();
      const double size = it.getSize();

      collect_face(0, 1, is_occupied_at(tree, cx + size, cy, cz), cx, cy, cz, size);
      collect_face(0, -1, is_occupied_at(tree, cx - size, cy, cz), cx, cy, cz, size);
      collect_face(1, 1, is_occupied_at(tree, cx, cy + size, cz), cx, cy, cz, size);
      collect_face(1, -1, is_occupied_at(tree, cx, cy - size, cz), cx, cy, cz, size);
      collect_face(2, 1, is_occupied_at(tree, cx, cy, cz + size), cx, cy, cz, size);
      collect_face(2, -1, is_occupied_at(tree, cx, cy, cz - size), cx, cy, cz, size);
    }

    const std::size_t max_chunk_triangles = std::max<std::size_t>(2, max_triangles_);
    MarkerMsg marker = make_base_marker(msg, ns, 1, MarkerMsg::ADD);
    marker.points.reserve(max_chunk_triangles * 3);

    std::size_t chunk_id = 1;
    std::size_t merged_quads = 0;
    std::size_t emitted_triangles = 0;
    std::size_t chunk_triangles = 0;
    std::size_t published_chunks = 0;

    auto publish_chunk = [&]() {
        if (marker.points.empty())
        {
          return;
        }
        queue_marker(marker);
        ++published_chunks;
        ++chunk_id;
        marker = make_base_marker(msg, ns, static_cast<int>(chunk_id), MarkerMsg::ADD);
        marker.points.reserve(max_chunk_triangles * 3);
        chunk_triangles = 0;
      };

    auto add_surface_rect = [&](const PlaneKey & plane_key, double plane, double u0, double u1,
                                double v0, double v1) {
        if (chunk_triangles + 2 > max_chunk_triangles)
        {
          publish_chunk();
        }
        add_merged_face(marker, plane_key.axis, plane_key.side, plane, u0, u1, v0, v1);
        chunk_triangles += 2;
        emitted_triangles += 2;
        ++merged_quads;
      };

    std::vector<PlaneKey> plane_keys;
    plane_keys.reserve(planes.size());
    for (const auto & entry : planes)
    {
      plane_keys.push_back(entry.first);
    }
    std::sort(plane_keys.begin(), plane_keys.end());

    for (const PlaneKey & plane_key : plane_keys)
    {
      auto plane_it = planes.find(plane_key);
      if (plane_it == planes.end())
      {
        continue;
      }

      const FaceCellMap & cells = plane_it->second;
      std::vector<CellKey> sorted_cells;
      sorted_cells.reserve(cells.size());
      for (const auto & cell_entry : cells)
      {
        sorted_cells.push_back(cell_entry.first);
      }
      std::sort(sorted_cells.begin(), sorted_cells.end());

      std::unordered_set<CellKey, CellKeyHash> visited;
      visited.reserve(cells.size());

      for (const CellKey & start : sorted_cells)
      {
        if (visited.find(start) != visited.end())
        {
          continue;
        }
        if (cells.find(start) == cells.end())
        {
          continue;
        }

        std::int64_t width = 1;
        while (true)
        {
          CellKey candidate{start.u + width, start.v};
          if (visited.find(candidate) != visited.end() || cells.find(candidate) == cells.end())
          {
            break;
          }
          ++width;
        }

        std::int64_t height = 1;
        while (true)
        {
          bool row_ok = true;
          for (std::int64_t du = 0; du < width; ++du)
          {
            CellKey candidate{start.u + du, start.v + height};
            if (visited.find(candidate) != visited.end() || cells.find(candidate) == cells.end())
            {
              row_ok = false;
              break;
            }
          }
          if (!row_ok)
          {
            break;
          }
          ++height;
        }

        double plane = 0.0;
        double u_min = std::numeric_limits<double>::max();
        double u_max = std::numeric_limits<double>::lowest();
        double v_min = std::numeric_limits<double>::max();
        double v_max = std::numeric_limits<double>::lowest();
        bool has_cell = false;

        for (std::int64_t dv = 0; dv < height; ++dv)
        {
          for (std::int64_t du = 0; du < width; ++du)
          {
            CellKey key{start.u + du, start.v + dv};
            visited.insert(key);
            const FaceCell & cell = cells.at(key);
            if (!has_cell)
            {
              plane = cell.plane;
              has_cell = true;
            }
            u_min = std::min(u_min, cell.u_min);
            u_max = std::max(u_max, cell.u_max);
            v_min = std::min(v_min, cell.v_min);
            v_max = std::max(v_max, cell.v_max);
          }
        }

        if (has_cell && u_max > u_min && v_max > v_min)
        {
          add_surface_rect(plane_key, plane, u_min, u_max, v_min, v_max);
        }
      }
    }

    publish_chunk();
    for (std::size_t stale_id = published_chunks + 1; stale_id <= last_published_chunks_; ++stale_id)
    {
      queue_marker(make_base_marker(msg, ns, static_cast<int>(stale_id), MarkerMsg::DELETE));
    }
    last_published_chunks_ = published_chunks;

    RCLCPP_INFO(
      get_logger(),
      "Published greedy octomap surface marker stream: occupied=%zu visited_voxels=%zu "
      "exposed_faces=%zu merged_quads=%zu chunks=%zu triangles=%zu chunk_triangle_cap=%zu "
      "style=%s frame=%s",
      occupied_count,
      visited_voxels,
      exposed_cell_faces,
      merged_quads,
      published_chunks,
      emitted_triangles,
      max_chunk_triangles,
      marker_style_name(marker_style_),
      msg.header.frame_id.empty() ? "map" : msg.header.frame_id.c_str());
  }

  static void add_cube(MarkerMsg & marker, const PointMsg (&p)[8])
  {
    add_face(marker, p[4], p[6], p[7], p[5]);
    add_face(marker, p[0], p[1], p[3], p[2]);
    add_face(marker, p[2], p[3], p[7], p[6]);
    add_face(marker, p[0], p[4], p[5], p[1]);
    add_face(marker, p[1], p[5], p[7], p[3]);
    add_face(marker, p[0], p[2], p[6], p[4]);
  }

  void publish_full_voxel_markers(
    const OctomapMsg & msg,
    const octomap::OcTree & tree,
    std::size_t occupied_count)
  {
    constexpr const char * ns = "uav_sim_octomap_voxels_chunked";
    publish_initial_clear_once(msg, ns);

    const std::size_t max_chunk_triangles = std::max<std::size_t>(12, max_triangles_);
    const std::size_t max_voxels_per_chunk = std::max<std::size_t>(1, max_chunk_triangles / 12);
    MarkerMsg marker = make_base_marker(msg, ns, 1, MarkerMsg::ADD);
    marker.points.reserve(max_voxels_per_chunk * 36);

    std::size_t chunk_id = 1;
    std::size_t emitted_voxels = 0;
    std::size_t emitted_triangles = 0;
    std::size_t chunk_voxels = 0;
    std::size_t published_chunks = 0;

    auto publish_chunk = [&]() {
        if (marker.points.empty())
        {
          return;
        }
        queue_marker(marker);
        ++published_chunks;
        ++chunk_id;
        marker = make_base_marker(msg, ns, static_cast<int>(chunk_id), MarkerMsg::ADD);
        marker.points.reserve(max_voxels_per_chunk * 36);
        chunk_voxels = 0;
      };

    for (auto it = tree.begin_leafs(), end = tree.end_leafs(); it != end; ++it)
    {
      if (!tree.isNodeOccupied(*it))
      {
        continue;
      }

      if (chunk_voxels >= max_voxels_per_chunk)
      {
        publish_chunk();
      }

      const double cx = it.getX();
      const double cy = it.getY();
      const double cz = it.getZ();
      const double half = it.getSize() * voxel_scale_ * 0.5;
      const double x0 = cx - half;
      const double x1 = cx + half;
      const double y0 = cy - half;
      const double y1 = cy + half;
      const double z0 = cz - half;
      const double z1 = cz + half;
      const PointMsg p[8] = {
        point(x0, y0, z0),
        point(x0, y0, z1),
        point(x0, y1, z0),
        point(x0, y1, z1),
        point(x1, y0, z0),
        point(x1, y0, z1),
        point(x1, y1, z0),
        point(x1, y1, z1),
      };

      add_cube(marker, p);
      ++chunk_voxels;
      ++emitted_voxels;
      emitted_triangles += 12;
    }

    publish_chunk();
    for (std::size_t stale_id = published_chunks + 1; stale_id <= last_published_chunks_; ++stale_id)
    {
      queue_marker(make_base_marker(msg, ns, static_cast<int>(stale_id), MarkerMsg::DELETE));
    }
    last_published_chunks_ = published_chunks;

    RCLCPP_INFO(
      get_logger(),
      "Published full octomap voxel marker stream: occupied=%zu emitted_voxels=%zu chunks=%zu "
      "triangles=%zu chunk_triangle_cap=%zu style=%s frame=%s",
      occupied_count,
      emitted_voxels,
      published_chunks,
      emitted_triangles,
      max_chunk_triangles,
      marker_style_name(marker_style_),
      msg.header.frame_id.empty() ? "map" : msg.header.frame_id.c_str());
  }

  std::string input_topic_;
  std::string output_topic_;
  std::size_t max_triangles_;
  double alpha_;
  double voxel_scale_;
  MarkerStyle marker_style_;
  std::chrono::duration<double> min_update_interval_;
  std::chrono::milliseconds chunk_publish_period_;
  std::size_t chunks_per_tick_;
  std::size_t last_published_chunks_ = 0;
  std::size_t last_subscription_count_ = 0;
  std::uint64_t latest_signature_ = 0;
  std::uint64_t last_built_signature_ = 0;
  bool sent_initial_clear_ = false;
  bool pending_rebuild_ = false;
  bool has_last_build_time_ = false;
  std::chrono::steady_clock::time_point last_build_time_{};
  OctomapMsg::SharedPtr latest_msg_;
  std::deque<MarkerMsg> pending_markers_;
  rclcpp::Subscription<OctomapMsg>::SharedPtr subscription_;
  rclcpp::Publisher<MarkerMsg>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::string input_topic = "/uav_sim/octomap_binary";
  std::string output_topic = "/map_3d_octomap_mesh";
  std::size_t max_triangles = 3000;
  double alpha = 0.95;
  double voxel_scale = 2.0;
  double min_update_interval_sec = 1.0;
  int chunk_publish_period_ms = 50;
  std::size_t chunks_per_tick = 1;
  std::string marker_style = "surface_mesh";

  for (int i = 1; i < argc; ++i)
  {
    const std::string arg = argv[i];
    auto next = [&]() -> std::string {
        if (i + 1 >= argc)
        {
          throw std::runtime_error("Missing value for " + arg);
        }
        return argv[++i];
      };

    if (arg == "--input-topic")
    {
      input_topic = next();
    }
    else if (arg == "--output-topic")
    {
      output_topic = next();
    }
    else if (arg == "--max-triangles")
    {
      max_triangles = static_cast<std::size_t>(std::stoul(next()));
    }
    else if (arg == "--alpha")
    {
      alpha = std::stod(next());
    }
    else if (arg == "--voxel-scale")
    {
      voxel_scale = std::stod(next());
    }
    else if (arg == "--style")
    {
      marker_style = next();
    }
    else if (arg == "--min-update-interval-sec")
    {
      min_update_interval_sec = std::stod(next());
    }
    else if (arg == "--chunk-publish-period-ms")
    {
      chunk_publish_period_ms = std::stoi(next());
    }
    else if (arg == "--chunks-per-tick")
    {
      chunks_per_tick = static_cast<std::size_t>(std::stoul(next()));
    }
  }

  auto node = std::make_shared<OctomapMarkerRelay>(
    input_topic,
    output_topic,
    max_triangles,
    alpha,
    voxel_scale,
    marker_style,
    min_update_interval_sec,
    chunk_publish_period_ms,
    chunks_per_tick);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
