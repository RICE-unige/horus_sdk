#include "horus/remote_rendering/protocol.hpp"
#include <cassert>
#include <cmath>
#include <fstream>
#include <future>
#include <iostream>
using namespace horus::remote_rendering;
template <class F> void rejects(F f) {
    bool failed = false;
    try {
        f();
    } catch (const std::exception &) {
        failed = true;
    }
    assert(failed);
}
std::vector<std::uint8_t> unhex(const std::string &s) {
    std::vector<std::uint8_t> out;
    for (std::size_t i = 0; i < s.size(); i += 2)
        out.push_back(std::stoul(s.substr(i, 2), nullptr, 16));
    return out;
}
void close(const Matrix4 &m, const Json &reference) {
    for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 4; ++c)
            assert(std::abs(m[r][c] - reference[r][c].get<double>()) < 2e-7);
}
int main() {
    const auto f = Json::parse(std::ifstream(HORUS_REMOTE_PROTOCOL_FIXTURE));
    const auto view = ViewState::from_dict(f["view"]);
    assert(view.session_id == "operator-a" && view.viewport_width == 1832);
    const auto round = ViewState::from_json(view.to_dict().dump());
    assert(round.sample_time_ns == view.sample_time_ns);
    close(eye_view_matrix(view.left_eye), f["matrices"]["left_view"]);
    close(eye_projection_matrix(view.left_eye, .1, 40), f["matrices"]["left_projection"]);
    close(eye_view_projection_matrix(view.left_eye, .1, 40), f["matrices"]["left_vp"]);
    close(eye_view_projection_matrix(view.right_eye, .1, 40), f["matrices"]["right_vp"]);
    auto p = eye_fov_from_unity_projection(1.31, 1.47, .08, -.04).renderer_projection();
    assert(std::abs(p[2] + .08) < 1e-12 && std::abs(p[3] - .04) < 1e-12);
    auto invalid = f["view"];
    invalid["workspace"]["scale"] = -1;
    rejects([&] { ViewState::from_dict(invalid); });
    invalid = f["view"];
    invalid["sample_time_ns"] = -1;
    rejects([&] { ViewState::from_dict(invalid); });
    invalid = f["view"];
    invalid["left_eye"]["rotation"] = {0, 0, 0, 0};
    rejects([&] { ViewState::from_dict(invalid); });
    rejects([&] { eye_projection_matrix(view.left_eye, 0, 40); });
    const auto wire = unhex(f["wire_hex"]), image = unhex(f["image_hex"]);
    const auto decoded = unpack_ros_diagnostic_frame(wire);
    assert(decoded.metadata == f["metadata"] &&
           std::equal(image.begin(), image.end(), decoded.image.begin(), decoded.image.end()));
    const auto native = pack_ros_diagnostic_frame(decoded.metadata, decoded.image);
    assert(unpack_ros_diagnostic_frame(native).metadata == decoded.metadata);
    assert(decoded.image.data() >= wire.data() && decoded.image.data() < wire.data() + wire.size());
    for (std::size_t n = 0; n < wire.size(); ++n)
        rejects([&] { unpack_ros_diagnostic_frame(std::span(wire).first(n)); });
    auto corrupt = wire;
    corrupt[5] = 1;
    rejects([&] { unpack_ros_diagnostic_frame(corrupt); });
    corrupt = wire;
    corrupt[8] = corrupt[9] = corrupt[10] = corrupt[11] = 255;
    rejects([&] { unpack_ros_diagnostic_frame(corrupt); });
    invalid = f["metadata"];
    invalid["codec"] = "h264";
    rejects([&] { validate_frame_metadata(invalid); });
    invalid["color_range"] = "limited";
    invalid["frame_marker_height"] = 8;
    assert(validate_frame_metadata(invalid)["codec"] == "h264");
    ClockAlignment clock;
    rejects([&] { clock.offset_ns(); });
    for (const auto &s : f["clock_samples"])
        clock.add({s[0], s[1], s[2], s[3]});
    assert(clock.offset_ns() == f["offset_ns"] && clock.round_trip_ns() == f["round_trip_ns"]);
    rejects([&] { clock.add({100, 20, 10, 101}); });
    ViewStreamSequenceGate gate;
    assert(!gate.accept(view, 100));
    assert(gate.accept(view, 101) == "stale_sample");
    auto next = view;
    next.view_stream_id = "stream-b";
    next.view_sequence = 1;
    next.sample_time_ns = 10;
    assert(gate.accept(next, 101) == "view_stream_busy");
    assert(!gate.accept(next, 103));
    assert(gate.accept(view, 106) == "retired_view_stream");
    assert(gate.revision() == 2);
    rejects([&] { gate.accept(next, 1); });
    LatestValueQueue<std::vector<int>> queue;
    queue.put({1});
    queue.put({2, 3});
    assert(queue.replaced_count() == 1);
    assert(queue.get()->size() == 2);
    assert(!queue.get(std::chrono::milliseconds(1)));
    auto waiter = std::async(std::launch::async, [&] { return queue.get(); });
    queue.close();
    assert(!waiter.get());
    rejects([&] { queue.put({4}); });
    // Native framing output is also consumed by the Python cross-language test.
    for (auto b : native)
        std::cout << "0123456789abcdef"[b >> 4] << "0123456789abcdef"[b & 15];
    std::cout << '\n';
}
