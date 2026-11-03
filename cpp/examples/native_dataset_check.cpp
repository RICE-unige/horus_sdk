#include <horus/remote_rendering/datasets.hpp>
#include <iostream>
int main(int argc, char **argv) {
    using namespace horus::remote_rendering;
    try {
        if (argc != 4)
            throw std::invalid_argument("Usage: native_dataset_check MODE SOURCE OUTPUT_LOD");
        const std::string mode = argv[1];
        Json out, points = Json::array(), colors = Json::array();
        auto append = [&](const SceneAsset &s) {
            for (auto p : s.vertices)
                points.push_back(p);
            for (auto c : s.point_colors)
                colors.push_back(c);
        };
        if (mode == "cow")
            append(prepare_cow_lady_scene(argv[2], 1.7));
        else {
            std::unique_ptr<PointChunkStream> stream;
            if (mode == "eth3d") {
                out["scene"] = prepare_eth3d_scene("electro", argv[2], 1.7);
                stream = std::make_unique<PointChunkStream>(out["scene"], 17);
            } else
                stream = std::make_unique<PointChunkStream>(std::filesystem::path(argv[2]), 17);
            while (auto chunk = stream->next())
                append(*chunk);
            if (mode == "eth3d") {
                PointChunkStream source(out["scene"], 13);
                out["lod_count"] = write_point_lod(source, argv[3], .05f);
            } else {
                PointChunkStream source(std::filesystem::path(argv[2]), 13);
                out["lod_count"] = write_point_lod(source, argv[3], .05f);
            }
            PointChunkStream lod(std::filesystem::path(argv[3]), 7);
            size_t n = 0;
            while (auto chunk = lod.next())
                n += chunk->vertices.size();
            if (n != out["lod_count"].get<size_t>())
                throw std::runtime_error("LOD roundtrip mismatch");
        }
        out["points"] = points;
        out["colors"] = colors;
        out["catalog"] = scene_catalog("/scene-cache");
        std::cout << out.dump() << '\n';
    } catch (const std::exception &e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
