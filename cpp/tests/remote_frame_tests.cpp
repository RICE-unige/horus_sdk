#include "horus/remote_rendering/frame.hpp"
#include <cassert>
#include <cmath>
#include <fstream>
#include <limits>
#include <nlohmann/json.hpp>
using namespace horus::remote_rendering;
using Json = nlohmann::json;
template <class F> void rejects(F f) {
    bool rejected = false;
    try {
        f();
    } catch (const std::exception &) {
        rejected = true;
    }
    assert(rejected);
}
std::vector<uint8_t> unhex(const std::string &s) {
    std::vector<uint8_t> b;
    for (size_t i = 0; i < s.size(); i += 2)
        b.push_back(std::stoul(s.substr(i, 2), nullptr, 16));
    return b;
}
std::vector<float> values(const Json &j) {
    std::vector<float> out;
    for (auto &v : j)
        out.push_back(v.is_null() ? std::numeric_limits<float>::infinity() : v.get<float>());
    return out;
}
int main() {
    auto f = Json::parse(std::ifstream(HORUS_REMOTE_FRAME_FIXTURE));
    auto common = f["common"];
    DepthFrameInfo info;
    info.sequence = common["sequence"];
    info.capture_time_ns = common["capture_time_ns"];
    info.color_width = common["color_width"];
    info.color_height = common["color_height"];
    info.near_m = common["near_m"];
    info.far_m = common["far_m"];
    for (size_t eye = 0; eye < 2; ++eye) {
        info.eyes[eye].projection = f["projections"][eye].get<std::array<float, 4>>();
        info.eyes[eye].position = f["positions"][eye].get<std::array<float, 3>>();
        info.eyes[eye].rotation = f["rotations"][eye].get<std::array<float, 4>>();
    }
    for (auto &c : f["cases"]) {
        info.width = c["width"];
        info.height = c["height"];
        info.view_count = c["view_count"];
        info.mono_atlas = c["mono_atlas"];
        auto wire = unhex(c["wire_hex"]), native = pack_depth_frame(values(c["depth"]), info);
        assert(native == wire);
        auto frame = unpack_depth_frame(wire);
        assert(frame.depth_codes == c["codes"].get<std::vector<uint16_t>>());
        assert(frame.info.capture_time_ns == info.capture_time_ns &&
               frame.info.sequence == info.sequence);
        assert(frame.info.eyes[0].projection == info.eyes[0].projection);
        auto depths = dequantize_depth(frame.depth_codes, info.near_m, info.far_m);
        auto raw = values(c["depth"]);
        for (size_t i = 0; i < raw.size(); ++i)
            if (frame.depth_codes[i])
                assert(std::abs(raw[i] - depths[i]) < .0002f);
            else
                assert(std::isinf(depths[i]));
        for (size_t size = 0; size < wire.size(); ++size)
            rejects([&] { unpack_depth_frame(std::span(wire).first(size)); });
        wire.back() ^= 4;
        rejects([&] { unpack_depth_frame(wire); });
    }
    for (size_t index = 0; index < 3; ++index) {
        auto c = f["reductions"][index];
        auto reduced = reduce_depth(values(f["source"]), 8, 6, c["size"][0], c["size"][1],
                                    static_cast<DepthReduction>(index));
        assert(reduced == values(c["expected"]));
    }
    rejects([&] { reduce_depth(values(f["source"]), 8, 6, 3, 2, DepthReduction::Minimum); });
    rejects([&] { quantize_depth(std::array<float, 1>{1}, 1, 1); });
    rejects([&] { pack_depth_frame(std::array<float, 1>{1}, info); });
    auto depth = values(f["cases"][2]["depth"]);
    depth.back() = 1;
    rejects([&] { pack_depth_frame(depth, info); });
    std::vector<uint8_t> rgb(400 * 20 * 3, 127);
    embed_frame_marker(rgb, 400, 20, info.sequence);
    const auto row = unhex(f["marker_row_hex"]);
    for (size_t y = 0; y < 20; ++y)
        for (size_t x = 0; x < row.size(); ++x)
            assert(rgb[y * row.size() + x] == ((y < 8 || y >= 12) ? row[x] : 127));
    assert(decode_frame_marker(rgb, 400, 20) == 0xabcd);
    std::fill(rgb.begin(), rgb.end(), 0);
    rejects([&] { decode_frame_marker(rgb, 400, 20); });
    const std::vector<uint8_t> jpeg{0xff, 0xd8, 0xff, 0xd9}, frame{1, 2, 3};
    auto carrier = pack_ros_debug_frame(jpeg, frame);
    auto view = unpack_ros_debug_frame(carrier);
    assert(std::equal(view.jpeg.begin(), view.jpeg.end(), jpeg.begin(), jpeg.end()));
    assert(view.frame.data() == carrier.data() + 16);
    carrier.push_back(0);
    rejects([&] { unpack_ros_debug_frame(carrier); });
    for (size_t i = 0; i < 2; ++i) {
        auto format = static_cast<LumaDepthFormat>(i);
        auto encoded = encode_luma_depth(values(f["source"]), 8, 6, .1f, 10, format);
        assert(encoded == unhex(f["luma"][i]["encoded_hex"]));
        auto decoded = decode_luma_depth(encoded, 8, 6, .1f, 10, format);
        auto expected = values(f["luma"][i]["decoded"]);
        for (size_t j = 0; j < expected.size(); ++j)
            assert(decoded[j] == expected[j] || std::abs(decoded[j] - expected[j]) < 1e-6f);
    }
    DynamicCameraPose pose;
    auto p = f["dynamic_pose"];
    pose.position = p["position"].get<std::array<float, 3>>();
    pose.rotation = p["rotation"].get<std::array<float, 4>>();
    std::vector<uint8_t> atlas(72 * 32 * 3, 71);
    assert(encode_dynamic_pose_metadata(atlas, 72, 32, pose, 0xabcd, 40) == 32);
    assert(atlas == unhex(p["metadata_hex"]));
    auto decoded_pose = decode_dynamic_pose_metadata(atlas, 72, 32, 40);
    assert(decoded_pose.sequence == 0xabcd);
    for (size_t i = 0; i < 3; ++i)
        assert(std::abs(decoded_pose.pose.position[i] - p["decoded_position"][i].get<float>()) <
               1e-5);
    for (size_t i = 0; i < 4; ++i)
        assert(std::abs(decoded_pose.pose.rotation[i] - p["decoded_rotation"][i].get<float>()) <
               1e-6);
    auto packed =
        pack_dynamic_rgbd(std::vector<uint8_t>(72 * 32 * 3, 127), std::vector<float>(72 * 32, 1.5f),
                          72, 32, pose, 0xabcd, .1f, 10.f, 40.f);
    assert(packed == unhex(p["packed_hex"]));
    rejects([&] { encode_dynamic_pose_metadata(atlas, 72, 16, pose, 1, 40); });
    rejects([&] { metadata_rows(4); });
    rejects([&] { decode_luma_depth(atlas, 72, 32, 0, 0); });
}
