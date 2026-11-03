#include <cassert>
#include <horus/remote_rendering/scene.hpp>
#include <iostream>
using namespace horus::remote_rendering;
int main(int argc, char **argv) {
    if (argc == 2) {
        const auto s = load_scene(argv[1], {.bake_lighting = false});
        std::cout << Json{{"vertices", s.vertices},
                          {"faces", s.faces},
                          {"point_colors", s.point_colors},
                          {"face_colors", s.face_colors},
                          {"uvs", s.uvs},
                          {"texture_rects", s.texture_rects},
                          {"atlas", s.texture_atlas},
                          {"width", s.texture_width},
                          {"height", s.texture_height}}
                         .dump();
        return 0;
    }
    const auto root = std::filesystem::path(HORUS_SCENE_FIXTURE);
    auto points = load_scene(root / "points.ply", {.world_scale = 2, .bake_lighting = false});
    assert(points.vertices.size() == 3 && points.faces.empty());
    assert((points.vertices[0] == Point3f{2, 4, 6}));
    assert((points.point_colors[0] == Rgb8{255, 0, 0}));
    assert((points.point_colors[1] == Rgb8{0, 255, 0}));
    assert((points.point_colors[2] == Rgb8{0, 0, 255}));
    try {
        points.mesh_input();
        assert(false);
    } catch (const std::invalid_argument &) {
    }
    auto mesh = load_scene(root / "triangle.obj", {.bake_lighting = false});
    assert(mesh.faces.size() == 1 && mesh.vertices.size() == 3);
    assert((mesh.face_colors[0] == Rgb8{255, 0, 0}));
    assert((mesh.uvs[0] == std::array<float, 2>{0, 1}));
    auto input = mesh.mesh_input();
    assert(input.vertices.data() == mesh.vertices.data());
    assert(input.colors.size() == 1 && input.texture_atlas.size() == 256);
    try {
        load_scene(root / "triangle.obj", {.maximum_vertices = 2});
        assert(false);
    } catch (const std::length_error &) {
    }
    try {
        load_scene(root / "missing.obj");
        assert(false);
    } catch (const std::invalid_argument &) {
    }
    try {
        load_scene(root / "points.ply", {.world_scale = 0});
        assert(false);
    } catch (const std::invalid_argument &) {
    }
}
