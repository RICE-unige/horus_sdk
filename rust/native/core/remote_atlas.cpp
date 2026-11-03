#include "horus/remote_rendering/atlas.hpp"
#include "horus/remote_rendering/frame.hpp"
#include <Eigen/Geometry>
#include <numbers>
namespace horus::remote_rendering {
namespace {
void require(bool ok, const char *error) {
    if (!ok)
        throw std::invalid_argument(error);
}
struct Tile {
    uint32_t x, y, w, h;
    EyeView eye;
};
std::vector<Tile> tiles(std::span<const RemoteRenderViewSpec> views, const AtlasOptions &o) {
    require(o.width && o.height && o.width <= 16384 && o.height <= 16384 &&
                uint64_t(o.width) * o.height <= 16000000,
            "Atlas exceeds bounded dimensions");
    require(!views.empty() && views.size() <= 12, "Atlas requires 1..12 views");
    require(std::isfinite(o.world_scale) && o.world_scale > 0 &&
                std::isfinite(o.vertical_fov_degrees) && o.vertical_fov_degrees > 0 &&
                o.vertical_fov_degrees < 179,
            "Invalid atlas scale/FOV");
    require(std::isfinite(o.near_base) && std::isfinite(o.far_base) && o.near_base > 0 &&
                o.far_base > o.near_base && o.far_base * o.world_scale <= 1e9,
            "Invalid atlas depth range");
    require(o.hole_fill_iterations >= 0 && o.hole_fill_iterations <= 8 && o.point_radius >= 0 &&
                o.point_radius <= 4,
            "Invalid atlas raster settings");
    std::vector<Tile> out;
    uint64_t total_pixels = 0;
    for (const auto &v : views) {
        require(std::isfinite(v.atlas_x) && std::isfinite(v.atlas_y) &&
                    std::isfinite(v.atlas_width) && std::isfinite(v.atlas_height) &&
                    v.atlas_x >= 0 && v.atlas_y >= 0 && v.atlas_width > 0 && v.atlas_height > 0 &&
                    v.atlas_x + v.atlas_width <= 1.00000001 &&
                    v.atlas_y + v.atlas_height <= 1.00000001,
                "Invalid atlas rectangle");
        require(std::isfinite(v.yaw_degrees) && std::isfinite(v.pitch_degrees) &&
                    std::all_of(v.position.begin(), v.position.end(),
                                [](double x) { return std::isfinite(x); }),
                "Invalid atlas pose");
        Tile t;
        t.x = uint32_t(std::nearbyint(v.atlas_x * o.width));
        t.y = o.height - uint32_t(std::nearbyint((v.atlas_y + v.atlas_height) * o.height));
        t.w = uint32_t(std::nearbyint(v.atlas_width * o.width));
        t.h = uint32_t(std::nearbyint(v.atlas_height * o.height));
        require(t.w && t.h && uint64_t(t.x) + t.w <= o.width && uint64_t(t.y) + t.h <= o.height,
                "Rounded atlas rectangle is out of bounds");
        total_pixels += uint64_t(t.w) * t.h;
        require(total_pixels <= 16000000, "Combined atlas view targets exceed budget");
        const double pitch = v.pitch_degrees * std::numbers::pi / 180,
                     yaw = v.yaw_degrees * std::numbers::pi / 180;
        const Eigen::Quaterniond q = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitY()) *
                                     Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitX());
        t.eye.position = {v.position[0] * o.world_scale, v.position[1] * o.world_scale,
                          v.position[2] * o.world_scale};
        t.eye.rotation = {q.x(), q.y(), q.z(), q.w()};
        double f = std::tan(o.vertical_fov_degrees * std::numbers::pi / 360),
               aspect = double(t.w) / t.h;
        t.eye.fov = {-f * aspect, f * aspect, f, -f};
        out.push_back(t);
    }
    return out;
}
void fill(RenderedViews &r, int iterations) {
    for (int k = 0; k < iterations; k++) {
        auto rgb = r.rgb;
        auto depth = r.depth;
        for (int y = 0; y < int(r.height); y++)
            for (int x = 0; x < int(r.width); x++) {
                size_t at = size_t(y) * r.width + x;
                if (std::isfinite(r.depth[at]))
                    continue;
                for (auto d :
                     std::array<std::array<int, 2>, 4>{{{0, 1}, {0, -1}, {1, 0}, {-1, 0}}}) {
                    int xx = x + d[0], yy = y + d[1];
                    if (xx < 0 || yy < 0 || xx >= int(r.width) || yy >= int(r.height))
                        continue;
                    size_t source = size_t(yy) * r.width + xx;
                    if (r.depth[source] < depth[at]) {
                        depth[at] = r.depth[source];
                        std::copy_n(r.rgb.data() + source * 3, 3, rgb.data() + at * 3);
                    }
                }
            }
        r.rgb = std::move(rgb);
        r.depth = std::move(depth);
    }
}
template <class Next>
RenderedViews render(Next next, std::span<const RemoteRenderViewSpec> views,
                     const AtlasOptions &o) {
    const auto ts = tiles(views, o);
    std::vector<RenderedViews> targets;
    for (const auto &t : ts) {
        RenderedViews r;
        r.width = t.w;
        r.height = t.h;
        r.view_count = 1;
        r.rgb.resize(size_t(t.w) * t.h * 3);
        r.depth.assign(size_t(t.w) * t.h, INFINITY);
        targets.push_back(std::move(r));
    }
    while (auto chunk = next())
        for (size_t i = 0; i < ts.size(); i++) {
            const auto &t = ts[i];
            auto r =
                render_colored_points(chunk->vertices, chunk->point_colors, std::span(&t.eye, 1),
                                      {t.w, t.h, float(o.near_base * o.world_scale),
                                       float(o.far_base * o.world_scale), o.point_radius});
            for (size_t at = 0; at < r.depth.size(); at++)
                if (r.depth[at] < targets[i].depth[at]) {
                    targets[i].depth[at] = r.depth[at];
                    std::copy_n(r.rgb.data() + at * 3, 3, targets[i].rgb.data() + at * 3);
                }
        }
    RenderedViews out;
    out.width = o.width;
    out.height = o.height;
    out.view_count = 1;
    out.rgb.resize(size_t(o.width) * o.height * 3);
    out.depth.assign(size_t(o.width) * o.height, INFINITY);
    for (size_t i = 0; i < ts.size(); i++) {
        fill(targets[i], o.hole_fill_iterations);
        const auto &t = ts[i];
        for (uint32_t y = 0; y < t.h; y++) {
            size_t at = size_t(t.y + y) * o.width + t.x, src = size_t(y) * t.w;
            std::copy_n(targets[i].rgb.data() + src * 3, t.w * 3, out.rgb.data() + at * 3);
            std::copy_n(targets[i].depth.data() + src, t.w, out.depth.data() + at);
        }
    }
    return out;
}
} // namespace
RenderedViews render_multiview_atlas(PointChunkStream &stream,
                                     std::span<const RemoteRenderViewSpec> views,
                                     const AtlasOptions &o) {
    return render([&] { return stream.next(); }, views, o);
}
RenderedViews render_multiview_atlas(std::span<const Point3f> points, std::span<const Rgb8> colors,
                                     std::span<const RemoteRenderViewSpec> views,
                                     const AtlasOptions &o) {
    require(points.size() == colors.size() && points.size() <= 100000000,
            "Invalid point/color count");
    size_t offset = 0;
    return render(
        [&]() -> std::optional<SceneAsset> {
            if (offset == points.size())
                return {};
            const size_t n = std::min(size_t(1000000), points.size() - offset);
            SceneAsset c;
            c.vertices.assign(points.begin() + offset, points.begin() + offset + n);
            c.point_colors.assign(colors.begin() + offset, colors.begin() + offset + n);
            offset += n;
            return c;
        },
        views, o);
}
SceneAsset unproject_multiview_surfels(const RenderedViews &atlas,
                                       std::span<const RemoteRenderViewSpec> views,
                                       uint32_t columns, uint32_t rows, double scale, double fov) {
    AtlasOptions o;
    o.width = atlas.width;
    o.height = atlas.height;
    o.world_scale = scale;
    o.vertical_fov_degrees = fov;
    tiles(views, o);
    require(atlas.view_count == 1 && atlas.rgb.size() == size_t(o.width) * o.height * 3 &&
                atlas.depth.size() == size_t(o.width) * o.height,
            "Invalid atlas buffers");
    require(columns && rows && uint64_t(columns) * rows <= 20000000 / views.size(),
            "Surfel grid exceeds budget");
    const double tangent = std::tan(fov * std::numbers::pi / 360);
    SceneAsset result;
    for (const auto &v : views) {
        const double pitch = v.pitch_degrees * std::numbers::pi / 180,
                     yaw = v.yaw_degrees * std::numbers::pi / 180;
        const Eigen::Matrix3f rotation = (Eigen::AngleAxisf(float(yaw), Eigen::Vector3f::UnitY()) *
                                          Eigen::AngleAxisf(float(pitch), Eigen::Vector3f::UnitX()))
                                             .toRotationMatrix();
        const double aspect = v.atlas_width * o.width / std::max(v.atlas_height * o.height, 1.);
        for (uint32_t y = 0; y < rows; y++)
            for (uint32_t x = 0; x < columns; x++) {
                float u = (float(x) + .5f) / columns, w = (float(y) + .5f) / rows;
                // Match the float32 texture-coordinate grid used by Python and Quest.
                const float atlas_u = float(v.atlas_x) + u * float(v.atlas_width),
                            atlas_v = float(v.atlas_y) + w * float(v.atlas_height);
                auto px = std::clamp(int(std::floor(atlas_u * o.width)), 0, int(o.width) - 1);
                auto py = std::clamp(int(o.height) - 1 - int(std::floor(atlas_v * o.height)), 0,
                                     int(o.height) - 1);
                size_t at = size_t(py) * o.width + px;
                float d = atlas.depth[at];
                if (!std::isfinite(d))
                    continue;
                const Eigen::Vector3f local(float((u * 2 - 1) * d * tangent * aspect),
                                            float((w * 2 - 1) * d * tangent), d);
                const Eigen::Vector3f p = rotation * local;
                result.vertices.push_back({float(p.x() + v.position[0] * scale),
                                           float(p.y() + v.position[1] * scale),
                                           float(p.z() + v.position[2] * scale)});
                result.point_colors.push_back(
                    {atlas.rgb[at * 3], atlas.rgb[at * 3 + 1], atlas.rgb[at * 3 + 2]});
            }
    }
    return result;
}
std::vector<uint8_t> pack_atlas_rgbd(const RenderedViews &a, double near_m, double far_m) {
    require(a.width && a.height && uint64_t(a.width) * a.height <= 16000000 && a.view_count == 1 &&
                a.rgb.size() == size_t(a.width) * a.height * 3 &&
                a.depth.size() == size_t(a.width) * a.height,
            "Invalid atlas buffers");
    const auto encoded = encode_luma_depth(a.depth, a.width, a.height, float(near_m), float(far_m));
    std::vector<uint8_t> out(a.rgb.size() * 2);
    for (size_t y = 0; y < a.height; y++) {
        std::copy_n(a.rgb.data() + y * a.width * 3, a.width * 3, out.data() + y * a.width * 6);
        std::copy_n(encoded.data() + y * a.width * 3, a.width * 3,
                    out.data() + (y * 2 + 1) * a.width * 3);
    }
    return out;
}
} // namespace horus::remote_rendering
