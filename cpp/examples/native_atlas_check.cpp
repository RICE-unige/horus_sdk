#include "horus/remote_rendering/atlas.hpp"
#include <iostream>
int main() {
    using namespace horus::remote_rendering;
    try {
        Json j;
        std::cin >> j;
        auto points = j.at("points").get<std::vector<Point3f>>();
        auto colors = j.at("colors").get<std::vector<Rgb8>>();
        auto views = j.at("views").get<std::vector<RemoteRenderViewSpec>>();
        auto options = j.at("options").get<AtlasOptions>();
        const auto atlas = render_multiview_atlas(points, colors, views, options);
        auto surfels = unproject_multiview_surfels(atlas, views, 12, 8, options.world_scale,
                                                   options.vertical_fov_degrees);
        Json depth = Json::array();
        for (auto v : atlas.depth)
            depth.push_back(std::isfinite(v) ? Json(v) : Json(nullptr));
        std::cout << Json{{"rgb", atlas.rgb},
                          {"depth", depth},
                          {"points", surfels.vertices},
                          {"colors", surfels.point_colors},
                          {"packed", pack_atlas_rgbd(atlas, options.near_base * options.world_scale,
                                                     options.far_base * options.world_scale)}}
                         .dump()
                  << '\n';
    } catch (const std::exception &e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
