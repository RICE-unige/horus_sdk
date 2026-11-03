#include "mesh_source_api.h"
#include "horus/mapping/mesh_source.hpp"
#include <cstring>
#include <nlohmann/json.hpp>
namespace {
using namespace horus::mapping;
using Json = nlohmann::json;
using Batch = std::vector<MeshPage>;
struct State {
    MeshSourceOptions options;
    MeshSourceState source;
    State(MeshSourceOptions o) : options(o), source(o) {}
};
template <class F> auto guarded(char **error, F f) noexcept -> decltype(f()) {
    if (error)
        *error = nullptr;
    try {
        return f();
    } catch (const std::exception &e) {
        if (error)
            *error = strdup(e.what());
    } catch (...) {
        if (error)
            *error = strdup("Native mesh operation failed");
    }
    return {};
}
template <class T> std::span<const T> input(const void *p, std::size_t n) {
    if (n && !p)
        throw std::invalid_argument("Missing mesh source buffer");
    return {static_cast<const T *>(p), n};
}
State &state(void *h) {
    if (!h)
        throw std::invalid_argument("Missing mesh source state");
    return *static_cast<State *>(h);
}
MeshPose pose(const horus_mesh_pose &p) {
    return {{p.translation[0], p.translation[1], p.translation[2]},
            {p.rotation[0], p.rotation[1], p.rotation[2], p.rotation[3]}};
}
} // namespace
extern "C" {
void *horus_mesh_source_create(const char *text, char **error) {
    return guarded(error, [&]() -> void * {
        if (!text)
            throw std::invalid_argument("Missing mesh options");
        auto j = Json::parse(text);
        MeshSourceOptions o;
        o.map.frame_id = j.value("frame_id", std::string("map"));
        o.map.max_pages = j.value("max_pages", 1024u);
        o.map.max_payload_bytes = j.value("max_payload_bytes", std::size_t(1024 * 1024));
        o.map.max_cache_bytes = j.value("max_cache_bytes", std::size_t(256 * 1024 * 1024));
        if (j.contains("initial_epoch") && !j["initial_epoch"].is_null())
            o.map.initial_epoch = j["initial_epoch"].get<std::uint64_t>();
        o.triangles_per_page = j.value("triangles_per_page", std::size_t(4096));
        o.max_input_points = j.value("max_input_points", std::size_t(3000000));
        o.blocks_per_axis = j.value("blocks_per_axis", 2u);
        return new State(o);
    });
}
void horus_mesh_source_destroy(void *h) { delete static_cast<State *>(h); }
void *horus_mesh_source_markers(void *h, const horus_triangle_marker *items, size_t n, double now,
                                char **error) {
    return guarded(error, [&]() -> void * {
        auto &s = state(h);
        if (n > 1024)
            throw std::invalid_argument("Marker batch exceeds 1024");
        std::vector<TriangleMarker> markers;
        markers.reserve(n);
        std::map<std::pair<std::string, std::int64_t>, MeshPose> transforms;
        std::size_t points = 0;
        for (const auto &v : input<horus_triangle_marker>(items, n)) {
            if (v.point_count > s.options.max_input_points - points ||
                (v.color_count && v.color_count != v.point_count))
                throw std::invalid_argument("Marker input count mismatch or limit exceeded");
            points += v.point_count;
            if (!v.frame || !v.name_space)
                throw std::invalid_argument("Missing marker identity");
            TriangleMarker m;
            m.frame = v.frame;
            m.name_space = v.name_space;
            m.id = v.id;
            m.action = v.action;
            m.type = v.type;
            m.stamp_ns = v.stamp_ns;
            m.frame_locked = v.frame_locked != 0;
            m.lifetime_seconds = v.lifetime_seconds;
            m.pose = pose(v.pose);
            std::copy_n(v.scale, 3, m.scale.begin());
            std::copy_n(v.color, 4, m.color.begin());
            const auto p = input<std::array<double, 3>>(v.points, v.point_count);
            m.points.assign(p.begin(), p.end());
            const auto c = input<std::array<std::uint8_t, 4>>(v.colors, v.color_count);
            m.colors.assign(c.begin(), c.end());
            transforms[{m.frame, m.stamp_ns}] = pose(v.transform);
            markers.push_back(std::move(m));
        }
        return new Batch(s.source.apply_markers(
            markers, now, [&](const auto &f, auto stamp) { return transforms.at({f, stamp}); }));
    });
}
void *horus_mesh_source_blocks(void *h, const horus_nvblox_block *items, size_t n, int clear,
                               const char *frame, int64_t stamp, horus_mesh_pose transform,
                               char **error) {
    return guarded(error, [&]() -> void * {
        auto &s = state(h);
        if (!frame || n > 65536)
            throw std::invalid_argument("Invalid block batch");
        std::vector<NvbloxBlockUpdate> blocks;
        blocks.reserve(n);
        std::size_t vertices = 0;
        for (const auto &v : input<horus_nvblox_block>(items, n)) {
            NvbloxBlockUpdate b{{v.index[0], v.index[1], v.index[2]}, nullptr};
            if (!v.deleted) {
                if (v.vertex_count > s.options.max_input_points - vertices ||
                    v.index_count > 9 * s.options.max_input_points ||
                    (v.color_count && v.color_count != v.vertex_count))
                    throw std::invalid_argument("Block input count mismatch or limit exceeded");
                vertices += v.vertex_count;
                auto g = std::make_shared<BlockGeometry>();
                const auto p = input<std::array<float, 3>>(v.vertices, v.vertex_count);
                g->vertices.assign(p.begin(), p.end());
                const auto t = input<std::uint32_t>(v.triangles, v.index_count);
                g->triangles.assign(t.begin(), t.end());
                const auto c = input<std::array<std::uint8_t, 4>>(v.colors, v.color_count);
                g->colors.assign(c.begin(), c.end());
                b.geometry = std::move(g);
            }
            blocks.push_back(std::move(b));
        }
        return new Batch(s.source.apply_blocks(
            blocks, clear != 0, frame, stamp, [&](const auto &, auto) { return pose(transform); }));
    });
}
void *horus_mesh_source_pages(void *h, const char *operation, const char *request, double now,
                              char **error) {
    return guarded(error, [&]() -> void * {
        auto &s = state(h);
        if (!operation)
            throw std::invalid_argument("Missing operation");
        const std::string op(operation);
        if (op == "snapshot")
            return new Batch(s.source.state().snapshot_updates());
        if (op == "expire")
            return new Batch(s.source.expire(now));
        if (op == "repair") {
            if (!request || std::strlen(request) > 256 * 1024)
                throw std::invalid_argument("Invalid repair request");
            return new Batch(s.source.state().repair_updates(request));
        }
        throw std::invalid_argument("Unknown source operation");
    });
}
char *horus_mesh_source_status(void *h, char **error) {
    return guarded(error, [&]() -> char * {
        const auto &s = state(h).source;
        const auto &m = s.state();
        return strdup(Json{{"manifest", Json::parse(m.manifest_json())},
                           {"page_count", m.page_count()},
                           {"cached_bytes", m.cached_bytes()},
                           {"source_bytes", s.source_bytes()},
                           {"unchanged_pages", m.unchanged_pages()},
                           {"suppressed_bytes", m.suppressed_bytes()}}
                          .dump()
                          .c_str());
    });
}
size_t horus_mesh_batch_size(void *h) { return h ? static_cast<Batch *>(h)->size() : 0; }
const uint8_t *horus_mesh_batch_page(void *h, size_t i, size_t *bytes) {
    if (bytes)
        *bytes = 0;
    if (!h || !bytes || i >= static_cast<Batch *>(h)->size())
        return nullptr;
    const auto &p = (*static_cast<Batch *>(h))[i]->payload;
    *bytes = p.size();
    return p.data();
}
void horus_mesh_batch_destroy(void *h) { delete static_cast<Batch *>(h); }
}
