#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>

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
    const std::string & marker_style)
  : Node("uav_sim_octomap_marker_relay"),
    input_topic_(input_topic),
    output_topic_(output_topic),
    max_triangles_(std::max<std::size_t>(12, max_triangles)),
    alpha_(std::clamp(alpha, 0.05, 1.0)),
    voxel_scale_(std::clamp(voxel_scale, 0.25, 8.0)),
    marker_style_(parse_marker_style(marker_style))
  {
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    publisher_ = create_publisher<MarkerMsg>(output_topic_, qos);
    subscription_ = create_subscription<OctomapMsg>(
      input_topic_,
      qos,
      [this](const OctomapMsg::SharedPtr msg) { handle_octomap(*msg); });

    RCLCPP_INFO(
      get_logger(),
      "Relaying %s octomap_msgs/Octomap to %s visualization_msgs/Marker TRIANGLE_LIST "
      "style=%s max_triangles=%zu voxel_scale=%.2f",
      input_topic_.c_str(),
      output_topic_.c_str(),
      marker_style_name(marker_style_),
      max_triangles_,
      voxel_scale_);
  }

private:
  void handle_octomap(const OctomapMsg & msg)
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

    if (marker_style_ == MarkerStyle::RvizVoxels)
    {
      publish_full_voxel_markers(msg, *tree, occupied_count);
      return;
    }

    publish_surface_mesh_markers(msg, *tree, occupied_count);
    return;

    const std::size_t max_voxels = std::max<std::size_t>(1, max_triangles_ / 12);
    const std::size_t stride = std::max<std::size_t>(
      1,
      static_cast<std::size_t>(std::ceil(static_cast<double>(occupied_count) / max_voxels)));

    MarkerMsg marker;
    marker.header = msg.header;
    if (marker.header.frame_id.empty())
    {
      marker.header.frame_id = "map";
    }
    marker.header.stamp = now();
    marker.ns = marker_style_ == MarkerStyle::SurfaceMesh
      ? "uav_sim_octomap_surface"
      : "uav_sim_octomap_voxels";
    marker.id = 0;
    marker.type = MarkerMsg::TRIANGLE_LIST;
    marker.action = MarkerMsg::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
    marker.color.r = 0.62f;
    marker.color.g = 0.72f;
    marker.color.b = 0.86f;
    marker.color.a = static_cast<float>(alpha_);

    std::size_t seen_occupied = 0;
    std::size_t sampled_voxels = 0;
    std::size_t emitted_triangles = 0;

    for (auto it = tree->begin_leafs(), end = tree->end_leafs(); it != end; ++it)
    {
      if (!tree->isNodeOccupied(*it))
      {
        continue;
      }

      if ((seen_occupied++ % stride) != 0)
      {
        continue;
      }

      const double cx = it.getX();
      const double cy = it.getY();
      const double cz = it.getZ();
      const double size = it.getSize();
      if (marker_style_ == MarkerStyle::RvizVoxels && emitted_triangles + 12 > max_triangles_)
      {
        break;
      }

      const double visual_size = marker_style_ == MarkerStyle::SurfaceMesh
        ? size * voxel_scale_ * std::max(1.0, std::cbrt(static_cast<double>(stride)))
        : size * voxel_scale_;
      const double half = visual_size * 0.5;
      const double x0 = cx - half;
      const double x1 = cx + half;
      const double y0 = cy - half;
      const double y1 = cy + half;
      const double z0 = cz - half;
      const double z1 = cz + half;

      const PointMsg p000 = point(x0, y0, z0);
      const PointMsg p001 = point(x0, y0, z1);
      const PointMsg p010 = point(x0, y1, z0);
      const PointMsg p011 = point(x0, y1, z1);
      const PointMsg p100 = point(x1, y0, z0);
      const PointMsg p101 = point(x1, y0, z1);
      const PointMsg p110 = point(x1, y1, z0);
      const PointMsg p111 = point(x1, y1, z1);

      auto maybe_add_face = [&](bool neighbor_occupied, const PointMsg & a, const PointMsg & b,
                                const PointMsg & c, const PointMsg & d) {
          if (neighbor_occupied || emitted_triangles + 2 > max_triangles_)
          {
            return;
          }
          add_face(marker, a, b, c, d);
          emitted_triangles += 2;
        };

      const bool cull_neighbors = marker_style_ == MarkerStyle::SurfaceMesh;
      maybe_add_face(cull_neighbors && is_occupied_at(*tree, cx + size, cy, cz), p100, p110, p111, p101);
      maybe_add_face(cull_neighbors && is_occupied_at(*tree, cx - size, cy, cz), p000, p001, p011, p010);
      maybe_add_face(cull_neighbors && is_occupied_at(*tree, cx, cy + size, cz), p010, p011, p111, p110);
      maybe_add_face(cull_neighbors && is_occupied_at(*tree, cx, cy - size, cz), p000, p100, p101, p001);
      maybe_add_face(cull_neighbors && is_occupied_at(*tree, cx, cy, cz + size), p001, p101, p111, p011);
      maybe_add_face(cull_neighbors && is_occupied_at(*tree, cx, cy, cz - size), p000, p010, p110, p100);

      ++sampled_voxels;
      if (emitted_triangles >= max_triangles_)
      {
        break;
      }
    }

    if (marker.points.empty())
    {
      RCLCPP_WARN(
        get_logger(),
        "Octomap conversion produced no visible surface triangles occupied=%zu sampled=%zu",
        occupied_count,
        sampled_voxels);
      return;
    }

    publisher_->publish(marker);
    RCLCPP_INFO(
      get_logger(),
      "Published octomap mesh marker: occupied=%zu sampled=%zu stride=%zu triangles=%zu points=%zu "
      "style=%s frame=%s",
      occupied_count,
      sampled_voxels,
      stride,
      emitted_triangles,
      marker.points.size(),
      marker_style_name(marker_style_),
      marker.header.frame_id.c_str());
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

    publisher_->publish(make_base_marker(msg, ns, 0, MarkerMsg::DELETEALL));
    sent_initial_clear_ = true;
    last_published_chunks_ = 0;
  }

  void publish_surface_mesh_markers(
    const OctomapMsg & msg,
    const octomap::OcTree & tree,
    std::size_t occupied_count)
  {
    constexpr const char * ns = "uav_sim_octomap_surface_chunked";
    publish_initial_clear_once(msg, ns);

    const std::size_t max_chunk_triangles = std::max<std::size_t>(2, max_triangles_);
    MarkerMsg marker = make_base_marker(msg, ns, 1, MarkerMsg::ADD);
    marker.points.reserve(max_chunk_triangles * 3);

    std::size_t chunk_id = 1;
    std::size_t visited_voxels = 0;
    std::size_t emitted_triangles = 0;
    std::size_t chunk_triangles = 0;
    std::size_t published_chunks = 0;

    auto publish_chunk = [&]() {
        if (marker.points.empty())
        {
          return;
        }
        publisher_->publish(marker);
        ++published_chunks;
        ++chunk_id;
        marker = make_base_marker(msg, ns, static_cast<int>(chunk_id), MarkerMsg::ADD);
        marker.points.reserve(max_chunk_triangles * 3);
        chunk_triangles = 0;
      };

    auto add_surface_face = [&](bool neighbor_occupied, const PointMsg & a, const PointMsg & b,
                                const PointMsg & c, const PointMsg & d) {
        if (neighbor_occupied)
        {
          return;
        }
        if (chunk_triangles + 2 > max_chunk_triangles)
        {
          publish_chunk();
        }
        add_face(marker, a, b, c, d);
        chunk_triangles += 2;
        emitted_triangles += 2;
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
      const double half = size * voxel_scale_ * 0.5;
      const double x0 = cx - half;
      const double x1 = cx + half;
      const double y0 = cy - half;
      const double y1 = cy + half;
      const double z0 = cz - half;
      const double z1 = cz + half;

      const PointMsg p000 = point(x0, y0, z0);
      const PointMsg p001 = point(x0, y0, z1);
      const PointMsg p010 = point(x0, y1, z0);
      const PointMsg p011 = point(x0, y1, z1);
      const PointMsg p100 = point(x1, y0, z0);
      const PointMsg p101 = point(x1, y0, z1);
      const PointMsg p110 = point(x1, y1, z0);
      const PointMsg p111 = point(x1, y1, z1);

      add_surface_face(is_occupied_at(tree, cx + size, cy, cz), p100, p110, p111, p101);
      add_surface_face(is_occupied_at(tree, cx - size, cy, cz), p000, p001, p011, p010);
      add_surface_face(is_occupied_at(tree, cx, cy + size, cz), p010, p011, p111, p110);
      add_surface_face(is_occupied_at(tree, cx, cy - size, cz), p000, p100, p101, p001);
      add_surface_face(is_occupied_at(tree, cx, cy, cz + size), p001, p101, p111, p011);
      add_surface_face(is_occupied_at(tree, cx, cy, cz - size), p000, p010, p110, p100);
    }

    publish_chunk();
    for (std::size_t stale_id = published_chunks + 1; stale_id <= last_published_chunks_; ++stale_id)
    {
      publisher_->publish(make_base_marker(msg, ns, static_cast<int>(stale_id), MarkerMsg::DELETE));
    }
    last_published_chunks_ = published_chunks;

    RCLCPP_INFO(
      get_logger(),
      "Published full octomap surface marker stream: occupied=%zu visited_voxels=%zu chunks=%zu "
      "triangles=%zu chunk_triangle_cap=%zu style=%s frame=%s",
      occupied_count,
      visited_voxels,
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
        publisher_->publish(marker);
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
      publisher_->publish(make_base_marker(msg, ns, static_cast<int>(stale_id), MarkerMsg::DELETE));
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
  std::size_t last_published_chunks_ = 0;
  bool sent_initial_clear_ = false;
  rclcpp::Subscription<OctomapMsg>::SharedPtr subscription_;
  rclcpp::Publisher<MarkerMsg>::SharedPtr publisher_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::string input_topic = "/uav_sim/octomap_binary";
  std::string output_topic = "/map_3d_octomap_mesh";
  std::size_t max_triangles = 120000;
  double alpha = 0.95;
  double voxel_scale = 2.0;
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
  }

  auto node = std::make_shared<OctomapMarkerRelay>(
    input_topic,
    output_topic,
    max_triangles,
    alpha,
    voxel_scale,
    marker_style);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
