#include "horus/remote_rendering/gpu.hpp"
#include <cmath>
#include <iostream>
using namespace horus::remote_rendering;
int main(int argc, char **argv) {
    if (argc != 3)
        return 2;
    const std::array<Point3f, 3> points{{{-.5f, 0, 2}, {0, 0, 2}, {.5f, 0, 2}}};
    const std::array<Rgb8, 3> colors{{{255, 0, 0}, {0, 255, 0}, {0, 0, 255}}};
    std::array<EyeView, 2> eyes;
    eyes[0].position.x = -.1;
    eyes[1].position.x = .1;
    GpuRenderOptions options{32, 24, .1f, 10.f, 1};
    CudaRenderer point(argv[1], points.size());
    point.upload(std::span(points).first(2), std::span(colors).first(2));
    bool rejected = false;
    try {
        point.render_views(eyes, options);
    } catch (const std::runtime_error &) {
        rejected = true;
    }
    if (!rejected)
        throw std::runtime_error("Partial GPU upload was rendered");
    point.upload(std::span(points).subspan(2), std::span(colors).subspan(2), 2);
    auto p = point.render_views(eyes, options);
    std::array<Point3f, 4> vertices{{{-1, -1, 2}, {1, -1, 2}, {1, 1, 2}, {-1, 1, 2}}};
    std::array<std::array<std::uint32_t, 3>, 2> faces{{{0, 1, 2}, {0, 2, 3}}};
    std::array<Rgb8, 2> tints{{{255, 80, 20}, {20, 80, 255}}};
    CudaRenderer mesh(argv[2], MeshInput{vertices, faces, tints});
    auto m = mesh.render_views(eyes, options);
    auto encode = [](const RenderedViews &r) {
        Json depth = Json::array();
        for (auto d : r.depth)
            depth.push_back(std::isfinite(d) ? Json(d) : Json(nullptr));
        return Json{{"rgb", r.rgb}, {"depth", depth}, {"cuda_ms", r.cuda_ms}};
    };
    std::cout << Json{{"points", encode(p)}, {"mesh", encode(m)}}.dump() << '\n';
}
