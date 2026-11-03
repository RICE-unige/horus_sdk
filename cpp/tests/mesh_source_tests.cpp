#include "horus/mapping/mesh_source.hpp"
#include <cassert>
#include <limits>
using namespace horus::mapping;
int main() {
    MeshSourceOptions options;
    options.map.initial_epoch = 42;
    options.triangles_per_page = 1;
    MeshSourceState state(options);
    TriangleMarker marker;
    marker.name_space = "room";
    marker.frame = "sensor";
    marker.id = 2;
    marker.pose.translation = {0, 2, 0};
    marker.scale = {2, 1, 1};
    marker.color = {10, 20, 30, 255};
    marker.points = {{{0, 0, 0}}, {{1, 0, 0}}, {{0, 1, 0}}};
    auto lookup = [](const std::string &frame, std::int64_t) {
        assert(frame == "sensor");
        MeshPose p;
        p.translation = {10, 0, 1};
        return p;
    };
    auto updates = state.apply_markers(std::span(&marker, 1), 0, lookup);
    assert(updates.size() == 1 && updates[0]->vertex_count == 3 && state.state().page_count() == 1);
    assert(state.apply_markers(std::span(&marker, 1), 0, lookup).empty());
    const auto bytes = state.state().snapshot_updates()[0]->payload;
    // Failed late marker leaves the entire array unapplied.
    auto other = marker;
    other.id = 3;
    other.points[0][0] = std::numeric_limits<double>::infinity();
    marker.points[0][0] = 5;
    std::array<TriangleMarker, 2> invalid{marker, other};
    bool failed = false;
    try {
        state.apply_markers(invalid, 0, lookup);
    } catch (const std::exception &) {
        failed = true;
    }
    assert(failed && state.state().snapshot_updates()[0]->payload == bytes);
    marker.points[0][0] = 0;
    auto clear = TriangleMarker{};
    clear.action = 3;
    std::array<TriangleMarker, 2> snapshot{clear, marker};
    assert(state.apply_markers(snapshot, 0, lookup).empty());
    marker.lifetime_seconds = 1;
    state.apply_markers(std::span(&marker, 1), 0, lookup);
    assert(state.expire(.5).empty());
    assert(state.expire(1).size() == 1 && state.state().page_count() == 0);
    auto geometry = std::make_shared<BlockGeometry>();
    geometry->vertices = {{{0, 0, 0}}, {{1, 0, 0}}, {{0, 1, 0}}};
    geometry->triangles = {0, 1, 2};
    geometry->colors = {{10, 20, 30, 255}, {10, 20, 30, 255}, {10, 20, 30, 255}};
    std::array<NvbloxBlockUpdate, 2> blocks{{{{-1, 0, 0}, geometry}, {{0, 0, 0}, geometry}}};
    updates = state.apply_blocks(blocks, false, "map", 0);
    assert(updates.size() == 2); // floor division across the origin, not C++ truncation
    assert(state.apply_blocks(blocks, false, "map", 0).empty());
    auto partial = blocks[1];
    partial.geometry.reset();
    assert(state.apply_blocks(std::span(&partial, 1), false, "map", 0).size() == 1);
    assert(state.state().page_count() == 1); // omitted block preserved
    failed = false;
    try {
        state.apply_blocks(blocks, false, "sensor", 0, lookup);
    } catch (const std::exception &) {
        failed = true;
    }
    assert(failed && state.state().page_count() == 1);
    updates = state.apply_blocks({}, true, "map", 0);
    assert(updates.size() == 1 && updates[0]->operation == MeshOperation::Clear &&
           state.source_bytes() == 0);
}
