#include "horus/mapping/mesh_source.hpp"
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <limits>
#include <nlohmann/json.hpp>
#include <tuple>

namespace horus::mapping {
namespace {
Eigen::Affine3d transform(const MeshPose &pose) {
    Eigen::Quaterniond q(pose.rotation[3], pose.rotation[0], pose.rotation[1], pose.rotation[2]);
    if (!q.coeffs().allFinite() || q.norm() < 1e-12)
        throw std::invalid_argument("Invalid mesh quaternion");
    Eigen::Vector3d t(pose.translation.data());
    if (!t.allFinite())
        throw std::invalid_argument("Invalid mesh translation");
    return Eigen::Translation3d(t) * q.normalized();
}
std::array<float, 3> point(const Eigen::Vector3d &p) {
    if (!p.allFinite() || p.cwiseAbs().maxCoeff() > std::numeric_limits<float>::max())
        throw std::invalid_argument("Mesh coordinates exceed finite float32 range");
    return {static_cast<float>(p.x()), static_cast<float>(p.y()), static_cast<float>(p.z())};
}
std::string frame(std::string value) {
    value.erase(0, value.find_first_not_of(" /\t\r\n"));
    auto last = value.find_last_not_of(" \t\r\n");
    if (last != std::string::npos)
        value.resize(last + 1);
    if (value.empty())
        throw std::invalid_argument("Missing source mesh frame");
    return value;
}
Eigen::Affine3d fixed_transform(const std::string &source, const std::string &target,
                                std::int64_t stamp, const MeshTransformLookup &lookup) {
    const auto input = frame(source);
    if (input == frame(target))
        return Eigen::Affine3d::Identity();
    if (!lookup)
        throw std::invalid_argument("No mesh transform from " + input + " to " + target);
    return transform(lookup(input, stamp));
}
std::string marker_key(const std::pair<std::string, std::int32_t> &key, std::size_t part) {
    return "marker:" + nlohmann::json::array({key.first, key.second, part}).dump();
}
std::array<std::int32_t, 3> page_key(std::array<std::int32_t, 3> key, int divisor) {
    for (auto &n : key)
        n = n / divisor - (n % divisor < 0 ? 1 : 0);
    return key;
}
std::string block_page(const std::array<std::int32_t, 3> &key) {
    return "nvblox:" + nlohmann::json(key).dump();
}
std::size_t geometry_bytes(const BlockGeometry &g) {
    return g.vertices.size() * sizeof(g.vertices[0]) + g.colors.size() * sizeof(g.colors[0]) +
           g.triangles.size() * sizeof(g.triangles[0]);
}
} // namespace
MeshSourceState::MeshSourceState(MeshSourceOptions options)
    : options_(std::move(options)), map_(options_.map) {
    if (!options_.triangles_per_page || options_.triangles_per_page > 20000 ||
        options_.max_input_points < 3 || options_.max_input_points > 20000000 ||
        !options_.blocks_per_axis || options_.blocks_per_axis > 8)
        throw std::invalid_argument("Invalid mesh source limits");
}
std::vector<MeshPage> MeshSourceState::apply_markers(std::span<const TriangleMarker> markers,
                                                     double now,
                                                     const MeshTransformLookup &lookup) {
    if (!std::isfinite(now) || markers.size() > 1024)
        throw std::invalid_argument("Invalid marker batch size or clock");
    std::size_t points = 0;
    for (const auto &m : markers) {
        if (m.points.size() > options_.max_input_points - points)
            throw std::length_error("Marker input point budget exceeded");
        points += m.points.size();
    }
    auto staged = *this;
    std::vector<MeshPage> updates;
    auto append = [&](std::vector<MeshPage> pages) {
        updates.insert(updates.end(), pages.begin(), pages.end());
    };
    auto erase = [&](const MarkerKey &key) {
        std::vector<std::pair<std::string, const BlockGeometry *>> changes;
        auto previous = staged.parts_.find(key);
        if (previous != staged.parts_.end()) {
            for (std::size_t p = 0; p < previous->second; ++p)
                changes.emplace_back(marker_key(key, p), nullptr);
            append(staged.map_.replace_many(changes));
            staged.parts_.erase(previous);
        }
        staged.expiry_.erase(key);
    };
    // DELETEALL + a complete array is a snapshot, not a reason to resend unchanged pages.
    std::size_t start = 0;
    for (std::size_t i = 0; i < markers.size(); ++i)
        if (markers[i].action == 3)
            start = i + 1;
    if (start && start < markers.size()) {
        std::set<MarkerKey> present;
        for (const auto &m : markers.subspan(start))
            if (m.action == 0)
                present.emplace(m.name_space, m.id);
        auto keys = staged.parts_;
        for (const auto &[key, _] : keys)
            if (!present.count(key))
                erase(key);
    } else
        start = 0;
    for (const auto &m : markers.subspan(start)) {
        const MarkerKey key{m.name_space, m.id};
        if (m.action == 3) {
            updates.push_back(staged.map_.clear());
            staged.parts_.clear();
            staged.expiry_.clear();
            staged.blocks_.clear();
            staged.page_blocks_.clear();
            staged.block_frame_.clear();
            staged.source_bytes_ = 0;
            continue;
        }
        if (m.action == 2) {
            erase(key);
            continue;
        }
        if (m.action != 0 || m.type != 11 || m.frame_locked)
            throw std::invalid_argument(
                "Mesh maps require non-frame-locked TRIANGLE_LIST ADD/DELETE/DELETEALL");
        if (m.points.size() % 3 || (!m.colors.empty() && m.colors.size() != m.points.size()))
            throw std::invalid_argument("Incomplete marker triangles or color count mismatch");
        if (!std::isfinite(m.lifetime_seconds) || m.lifetime_seconds < 0 ||
            !std::isfinite(now + m.lifetime_seconds))
            throw std::invalid_argument("Invalid marker lifetime");
        if (m.points.empty()) {
            erase(key);
            continue;
        }
        const auto matrix =
            fixed_transform(m.frame, options_.map.frame_id, m.stamp_ns, lookup) * transform(m.pose);
        const Eigen::Vector3d scale(m.scale.data());
        if (!scale.allFinite())
            throw std::invalid_argument("Invalid marker scale");
        const auto step = options_.triangles_per_page * 3;
        const auto count = (m.points.size() + step - 1) / step;
        if (count > options_.map.max_pages)
            throw std::length_error("Marker exceeds map page capacity");
        std::vector<BlockGeometry> pages(count);
        for (std::size_t p = 0; p < count; ++p) {
            auto &g = pages[p];
            using Vertex = std::pair<std::array<float, 3>, std::array<std::uint8_t, 4>>;
            std::map<Vertex, std::uint32_t> unique;
            for (std::size_t i = p * step; i < std::min((p + 1) * step, m.points.size()); ++i) {
                const auto xyz =
                    point(matrix * Eigen::Vector3d(m.points[i].data()).cwiseProduct(scale));
                const auto color = m.colors.empty() ? m.color : m.colors[i];
                auto [it, inserted] = unique.emplace(Vertex{xyz, color}, g.vertices.size());
                if (inserted) {
                    g.vertices.push_back(xyz);
                    g.colors.push_back(color);
                }
                g.triangles.push_back(it->second);
            }
        }
        std::vector<std::pair<std::string, const BlockGeometry *>> changes;
        const auto old = staged.parts_.find(key);
        for (std::size_t p = count; p < (old == staged.parts_.end() ? 0 : old->second); ++p)
            changes.emplace_back(marker_key(key, p), nullptr);
        for (std::size_t p = 0; p < count; ++p)
            changes.emplace_back(marker_key(key, p), &pages[p]);
        append(staged.map_.replace_many(changes));
        staged.parts_[key] = count;
        if (m.lifetime_seconds)
            staged.expiry_[key] = now + m.lifetime_seconds;
        else
            staged.expiry_.erase(key);
    }
    *this = std::move(staged);
    return updates;
}
std::vector<MeshPage> MeshSourceState::apply_blocks(std::span<const NvbloxBlockUpdate> blocks,
                                                    bool clear, const std::string &source_frame,
                                                    std::int64_t stamp,
                                                    const MeshTransformLookup &lookup) {
    if (blocks.size() > 65536)
        throw std::length_error("nvblox block count exceeds source limit");
    if (!clear && !blocks_.empty() && block_frame_ != frame(source_frame))
        throw std::invalid_argument("nvblox frame changed without map clear");
    const auto matrix = fixed_transform(source_frame, options_.map.frame_id, stamp, lookup);
    auto staged = *this;
    std::vector<MeshPage> updates;
    if (clear) {
        updates.push_back(staged.map_.clear());
        staged.blocks_.clear();
        staged.page_blocks_.clear();
        staged.parts_.clear();
        staged.expiry_.clear();
        staged.source_bytes_ = 0;
    }
    staged.block_frame_ = frame(source_frame);
    std::set<Key> dirty;
    std::size_t input_points = 0;
    for (const auto &b : blocks) {
        const auto page = page_key(b.index, options_.blocks_per_axis);
        staged.page_blocks_[page].erase(b.index);
        auto old = staged.blocks_.find(b.index);
        if (old != staged.blocks_.end()) {
            staged.source_bytes_ -= geometry_bytes(*old->second);
            staged.blocks_.erase(old);
        }
        if (b.geometry) {
            const auto &src = *b.geometry;
            if (src.vertices.size() > options_.max_input_points - input_points)
                throw std::length_error("nvblox point budget exceeded");
            input_points += src.vertices.size();
            if (src.triangles.size() % 3 || src.colors.size() != src.vertices.size())
                throw std::invalid_argument("Invalid nvblox geometry");
            if (geometry_bytes(src) > options_.map.max_cache_bytes - staged.source_bytes_)
                throw std::length_error("nvblox retained block budget exceeded");
            auto g = std::make_shared<BlockGeometry>();
            g->vertices.reserve(src.vertices.size());
            g->colors = src.colors;
            for (const auto &v : src.vertices)
                g->vertices.push_back(point(matrix * Eigen::Vector3d(v[0], v[1], v[2])));
            for (std::size_t i = 0; i < src.triangles.size(); i += 3) {
                const auto a = src.triangles[i], c = src.triangles[i + 1], d = src.triangles[i + 2];
                if (std::max({a, c, d}) >= g->vertices.size())
                    throw std::invalid_argument("nvblox triangle index outside geometry");
                if (a == c || c == d || a == d)
                    continue;
                g->triangles.insert(g->triangles.end(), {a, c, d});
            }
            if (!g->triangles.empty()) {
                staged.source_bytes_ += geometry_bytes(*g);
                staged.blocks_[b.index] = std::move(g);
                staged.page_blocks_[page].insert(b.index);
            }
        }
        dirty.insert(page_key(b.index, options_.blocks_per_axis));
    }
    if (staged.blocks_.size() > 65536)
        throw std::length_error("nvblox retained block count exceeded");
    std::map<Key, BlockGeometry> pages;
    for (const auto &key : dirty)
        pages[key];
    for (auto &[page, merged] : pages) {
        for (const auto &key : staged.page_blocks_[page]) {
            const auto &g = staged.blocks_.at(key);
            const auto base = merged.vertices.size();
            if (geometry_bytes(merged) + geometry_bytes(*g) >
                options_.map.max_payload_bytes - HMSH_V2_HEADER_SIZE)
                throw std::length_error("nvblox spatial page exceeds byte budget; reduce "
                                        "blocks_per_axis or increase page budget");
            merged.vertices.insert(merged.vertices.end(), g->vertices.begin(), g->vertices.end());
            merged.colors.insert(merged.colors.end(), g->colors.begin(), g->colors.end());
            for (auto index : g->triangles)
                merged.triangles.push_back(static_cast<std::uint32_t>(base + index));
        }
        if (staged.page_blocks_[page].empty())
            staged.page_blocks_.erase(page);
    }
    std::vector<std::pair<std::string, const BlockGeometry *>> changes;
    for (auto &[key, g] : pages)
        changes.emplace_back(block_page(key), g.triangles.empty() ? nullptr : &g);
    auto changes_out = staged.map_.replace_many(changes);
    updates.insert(updates.end(), changes_out.begin(), changes_out.end());
    *this = std::move(staged);
    return updates;
}
std::vector<MeshPage> MeshSourceState::expire(double now) {
    if (!std::isfinite(now))
        throw std::invalid_argument("Invalid marker expiry clock");
    std::vector<TriangleMarker> deletes;
    for (const auto &[key, deadline] : expiry_)
        if (now >= deadline) {
            TriangleMarker m;
            m.name_space = key.first;
            m.id = key.second;
            m.action = 2;
            deletes.push_back(std::move(m));
        }
    // Expiration may contain more than one source message's worth of keys.
    std::vector<MeshPage> updates;
    for (std::size_t i = 0; i < deletes.size(); i += 1024) {
        auto pages = apply_markers(
            std::span(deletes).subspan(i, std::min(std::size_t(1024), deletes.size() - i)), now);
        updates.insert(updates.end(), pages.begin(), pages.end());
    }
    return updates;
}
} // namespace horus::mapping
