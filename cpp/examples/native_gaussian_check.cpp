#include <fstream>
#include <horus/remote_rendering/gaussian.hpp>
#include <iostream>
int main(int argc, char **argv) {
    using namespace horus::remote_rendering;
    try {
        if (argc != 4 && argc != 6)
            throw std::invalid_argument(
                "Usage: native_gaussian_check PLY OPTIONS_JSON OUTPUT_JSON [PLUGIN OUTPUT_PREFIX]");
        std::ifstream input(argv[2]);
        auto cfg = Json::parse(input);
        auto s = load_gaussian_ply(
            argv[1], {.world_scale = cfg.value("world_scale", 1.),
                      .canonicalize_y_down = cfg.value("canonicalize_y_down", false)});
        auto vec = [](Vector3 v) { return Json::array({v.x, v.y, v.z}); };
        Json j = {{"means", s.means},
                  {"scales", s.scales},
                  {"quaternions", s.quaternions},
                  {"opacities", s.opacities},
                  {"spherical_harmonics", s.spherical_harmonics},
                  {"sh_degree", s.sh_degree},
                  {"discarded_nonfinite", s.discarded_nonfinite},
                  {"world_bounds_min", vec(s.world_bounds_min)},
                  {"world_bounds_max", vec(s.world_bounds_max)},
                  {"source_from_world_translation", vec(s.source_from_world_translation)}};
        if (argc == 6) {
            auto view = ViewState::from_dict(cfg.at("view"));
            GpuRenderOptions o{cfg.value("width", 256u), cfg.value("height", 192u), .05f, 100.f, 0};
            GaussianRenderer renderer(argv[4], s);
            auto r = renderer.render_views(std::array{view.left_eye, view.right_eye}, o);
            std::ofstream rgb(std::string(argv[5]) + ".rgb", std::ios::binary),
                depth(std::string(argv[5]) + ".depth", std::ios::binary);
            rgb.write(reinterpret_cast<const char *>(r.rgb.data()), r.rgb.size());
            depth.write(reinterpret_cast<const char *>(r.depth.data()),
                        r.depth.size() * sizeof(float));
            j["elapsed_ms"] = r.cuda_ms;
        }
        std::ofstream(argv[3]) << j.dump();
    } catch (const std::exception &e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
