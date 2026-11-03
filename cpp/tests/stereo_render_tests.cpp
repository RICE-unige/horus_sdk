#include "horus/remote_rendering/stereo.hpp"
#include <cmath>
#include <fstream>
#include <source_location>

using namespace horus::remote_rendering;
void check(bool condition, std::source_location where = std::source_location::current()) {
    if (!condition)
        throw std::runtime_error("Stereo contract mismatch at line " +
                                 std::to_string(where.line()));
}
std::string hex(std::span<const uint8_t> bytes) {
    std::string out;
    out.reserve(bytes.size() * 2);
    for (auto b : bytes) {
        out.push_back("0123456789abcdef"[b >> 4]);
        out.push_back("0123456789abcdef"[b & 15]);
    }
    return out;
}
std::vector<float> depth(const Json &values) {
    std::vector<float> out;
    for (const auto &v : values)
        out.push_back(v.is_null() ? INFINITY : v.get<float>());
    return out;
}
int main() {
    std::ifstream file(HORUS_STEREO_FIXTURE);
    Json fixture;
    file >> fixture;
    for (const auto &test : fixture["cases"]) {
        GpuRenderOptions o{test["width"], test["height"], .25f, 8.f, 0};
        StereoFrameContext c;
        c.session_id = "native";
        c.sequence = c.source_view_sequence = 42;
        c.workspace_revision = 3;
        c.predicted_display_time_ns = 100;
        c.render_start_ns = 10;
        c.render_complete_ns = 20;
        c.capture_time_ns = 30;
        c.depth_strip_height = test["strip"];
        c.marker_height = test["marker"];
        c.codec = test["codec"];
        const auto rgb = test["rgb"].get<std::vector<uint8_t>>();
        const auto d = depth(test["depth"]);
        auto frame = assemble_stereo_frame(rgb, d, o, c);
        check(hex(frame.stereo_rgba) == test["rgba_hex"].get<std::string>());
        check(hex(frame.video_rgb) == test["video_hex"].get<std::string>());
        check(frame.metadata == test["metadata"]);
        check(std::abs(frame.visibility_fraction - test["visibility"].get<double>()) < 1e-12);
        if (c.codec == "png") {
            auto packet = pack_stereo_diagnostic(frame);
            auto decoded = unpack_ros_diagnostic_frame(packet);
            check(decoded.metadata == frame.metadata);
            check(decoded.image.size() > 8 && hex(decoded.image.first(8)) == "89504e470d0a1a0a");
        }
        bool rejected = false;
        try {
            assemble_stereo_frame(std::span(rgb).first(rgb.size() - 1), d, o, c);
        } catch (const std::invalid_argument &) {
            rejected = true;
        }
        check(rejected);
        c.capture_time_ns = 19;
        rejected = false;
        try {
            assemble_stereo_frame(rgb, d, o, c);
        } catch (const std::invalid_argument &) {
            rejected = true;
        }
        check(rejected);
    }
    auto points = fixture["points"].get<std::vector<Point3f>>();
    auto colors = fixture["colors"].get<std::vector<Rgb8>>();
    std::array<EyeView, 2> eyes;
    for (size_t i = 0; i < 2; ++i) {
        eyes[i].position.x = i ? .1 : -.1;
        eyes[i].fov = {-4. / 3, 4. / 3, 1, -1};
    }
    auto rendered = render_colored_points(points, colors, eyes, {32, 24, .25f, 8.f, 1});
    check(hex(rendered.rgb) == fixture["cpu_rgb_hex"].get<std::string>());
    check(rendered.depth == depth(fixture["cpu_depth"]));
}
