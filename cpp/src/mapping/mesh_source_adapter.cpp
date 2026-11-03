#include "horus/mapping/mesh_source_adapter.hpp"
#include <stdexcept>
#ifdef HORUS_ROS2_ENABLED
#include "mesh_source_runtime.h"
#include "registry.h"
#endif
namespace horus::mapping {
struct MeshSourceAdapter::Impl {
#ifdef HORUS_ROS2_ENABLED
    std::unique_ptr<void, decltype(&horus_native_mesh_source_destroy)> runtime{
        nullptr, horus_native_mesh_source_destroy};
    static std::string take(char *text) {
        std::unique_ptr<char, decltype(&horus_native_free)> owned(text, horus_native_free);
        if (!text)
            throw std::runtime_error("Native mesh adapter allocation failed");
        return text;
    }
#endif
    explicit Impl(const MeshSourceAdapterOptions &o) {
#ifdef HORUS_ROS2_ENABLED
        auto input = nlohmann::json{{"input_topic", o.input_topic},
                                    {"input_type", o.input_type},
                                    {"output_topic", o.output_topic},
                                    {"frame_id", o.source.map.frame_id},
                                    {"max_pages", o.source.map.max_pages},
                                    {"max_payload_bytes", o.source.map.max_payload_bytes},
                                    {"max_cache_bytes", o.source.map.max_cache_bytes},
                                    {"triangles_per_page", o.source.triangles_per_page},
                                    {"max_input_points", o.source.max_input_points},
                                    {"blocks_per_axis", o.source.blocks_per_axis},
                                    {"pages_per_second", o.pages_per_second},
                                    {"bytes_per_second", o.bytes_per_second},
                                    {"use_sim_time", o.use_sim_time},
                                    {"color_mode", o.color_mode},
                                    {"fallback_color", o.fallback_color}}
                         .dump();
        char *error = nullptr;
        runtime.reset(horus_native_mesh_source_create(input.c_str(), &error));
        if (!runtime)
            throw std::runtime_error(take(error));
#else
        (void)o;
        throw std::runtime_error("Enable ROS 2 to receive mesh source topics");
#endif
    }
    nlohmann::json status() const {
#ifdef HORUS_ROS2_ENABLED
        return nlohmann::json::parse(take(horus_native_mesh_source_status(runtime.get())));
#else
        throw std::runtime_error("Enable ROS 2 to receive mesh source topics");
#endif
    }
};
MeshSourceAdapter::MeshSourceAdapter(const MeshSourceAdapterOptions &o)
    : impl_(std::make_unique<Impl>(o)) {}
MeshSourceAdapter::~MeshSourceAdapter() = default;
nlohmann::json MeshSourceAdapter::status() const { return impl_->status(); }
} // namespace horus::mapping
