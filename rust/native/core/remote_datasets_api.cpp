#include "horus/remote_rendering/datasets.hpp"
#include "remote_datasets.h"
#include <cstring>
using namespace horus::remote_rendering;
namespace {
template <class T, class F> T guarded(char **error, T failed, F f) {
    if (error)
        *error = nullptr;
    try {
        return f();
    } catch (const std::exception &e) {
        if (error)
            *error = strdup(e.what());
    } catch (...) {
        if (error)
            *error = strdup("Dataset operation failed");
    }
    return failed;
}
std::string text(const char *p, size_t n = 65536) {
    if (!p || strnlen(p, n + 1) > n)
        throw std::invalid_argument("Invalid dataset text");
    return p;
}
} // namespace
extern "C" {
char *horus_dataset_request(const char *request, char **error) {
    return guarded<char *>(error, nullptr, [&] {
        auto j = Json::parse(text(request, 1024 * 1024));
        const auto op = j.at("op").get<std::string>();
        Json out;
        if (op == "catalog")
            out = scene_catalog(j.value("cache_root", std::string{}));
        else if (op == "resolve")
            out = resolve_scene_asset(j.at("scene_id"), j.value("path", std::string{}),
                                      j.value("cache_root", std::string{}))
                      .string();
        else if (op == "eth3d")
            out = prepare_eth3d_scene(j.at("scene_id"), j.value("path", std::string{}),
                                      j.value("dataset_scale", 1.),
                                      j.value("cache_root", std::string{}));
        else
            throw std::invalid_argument("Unknown dataset operation");
        auto p = strdup(out.dump().c_str());
        if (!p)
            throw std::bad_alloc();
        return p;
    });
}
void *horus_dataset_cow(const char *path, double scale, char **error) {
    return guarded<void *>(
        error, nullptr, [&] { return new SceneAsset(prepare_cow_lady_scene(text(path), scale)); });
}
void *horus_dataset_stream(const char *descriptor, size_t chunk_size, char **error) {
    return guarded<void *>(error, nullptr, [&] {
        return new PointChunkStream(Json::parse(text(descriptor, 4 * 1024 * 1024)), chunk_size);
    });
}
void *horus_dataset_next(void *p, int *done, char **error) {
    return guarded<void *>(error, nullptr, [&]() -> void * {
        if (!p || !done)
            throw std::invalid_argument("Missing point stream");
        *done = 0;
        auto chunk = static_cast<PointChunkStream *>(p)->next();
        if (!chunk) {
            *done = 1;
            return nullptr;
        }
        return new SceneAsset(std::move(*chunk));
    });
}
uint64_t horus_dataset_write_lod(void *p, const char *path, float voxel, size_t maximum,
                                 char **error) {
    return guarded<uint64_t>(error, 0, [&] {
        if (!p)
            throw std::invalid_argument("Missing point stream");
        return write_point_lod(*static_cast<PointChunkStream *>(p), text(path), voxel, maximum);
    });
}
void horus_dataset_destroy(void *p) { delete static_cast<PointChunkStream *>(p); }
}
