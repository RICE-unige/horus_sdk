#include "horus/remote_rendering/protocol.hpp"
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <stdexcept>

namespace horus::remote_rendering {
namespace {
double finite(double x) {
    if (!std::isfinite(x))
        throw std::invalid_argument("Remote value must be finite");
    return x;
}
double number(const Json &j, const char *key) { return finite(j.at(key).get<double>()); }
std::uint64_t integer(const Json &j, const char *key,
                      std::uint64_t maximum = std::numeric_limits<std::int64_t>::max()) {
    const auto &v = j.at(key);
    if (!v.is_number_integer() ||
        (v.is_number_integer() && !v.is_number_unsigned() && v.get<std::int64_t>() < 0))
        throw std::invalid_argument(std::string(key) + " must be a non-negative integer");
    auto n = v.get<std::uint64_t>();
    if (n > maximum)
        throw std::invalid_argument(std::string(key) + " exceeds the native protocol range");
    return n;
}
std::string text(const Json &j, const char *key, bool required = true) {
    auto s = j.value(key, std::string{});
    auto first = s.find_first_not_of(" \t\r\n"), last = s.find_last_not_of(" \t\r\n");
    s = first == std::string::npos ? std::string{} : s.substr(first, last - first + 1);
    if (required && s.empty())
        throw std::invalid_argument(std::string(key) + " is required");
    return s;
}
Vector3 vector(const Json &j) {
    if (j.is_array() && j.size() == 3)
        return {finite(j[0]), finite(j[1]), finite(j[2])};
    return {number(j, "x"), number(j, "y"), number(j, "z")};
}
Quaternion quaternion(const Json &j) {
    Quaternion q;
    if (j.is_array() && j.size() == 4)
        q = {finite(j[0]), finite(j[1]), finite(j[2]), finite(j[3])};
    else
        q = {number(j, "x"), number(j, "y"), number(j, "z"), number(j, "w")};
    const auto norm = std::hypot(std::hypot(q.x, q.y), std::hypot(q.z, q.w));
    if (!std::isfinite(norm) || norm < 1e-8)
        throw std::invalid_argument("Invalid eye quaternion");
    return {q.x / norm, q.y / norm, q.z / norm, q.w / norm};
}
Json json(const Vector3 &v) { return {{"x", v.x}, {"y", v.y}, {"z", v.z}}; }
Json json(const Quaternion &q) { return {{"x", q.x}, {"y", q.y}, {"z", q.z}, {"w", q.w}}; }
Json json(const EyeFov &f) {
    return {{"left", f.left}, {"right", f.right}, {"up", f.up}, {"down", f.down}};
}
Json json(const EyeView &e) {
    return {{"position", json(e.position)}, {"rotation", json(e.rotation)}, {"fov", json(e.fov)}};
}
EyeView eye(const Json &j) {
    const auto &f = j.at("fov");
    EyeView e{vector(j.at("position")),
              quaternion(j.at("rotation")),
              {number(f, "left"), number(f, "right"), number(f, "up"), number(f, "down")}};
    e.fov.renderer_projection();
    return e;
}
Matrix4 matrix(const Eigen::Matrix4d &m) {
    Matrix4 out{};
    for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 4; ++c)
            out[r][c] = m(r, c);
    return out;
}
double median(std::vector<double> values) {
    if (values.empty())
        throw std::runtime_error("clock alignment has no samples");
    std::sort(values.begin(), values.end());
    const auto n = values.size();
    return n % 2 ? values[n / 2] : (values[n / 2 - 1] / 2 + values[n / 2] / 2);
}
std::uint32_t u32(std::span<const std::uint8_t> b, std::size_t o) {
    return std::uint32_t(b[o]) | (std::uint32_t(b[o + 1]) << 8) | (std::uint32_t(b[o + 2]) << 16) |
           (std::uint32_t(b[o + 3]) << 24);
}
void put32(std::vector<std::uint8_t> &b, std::uint32_t v) {
    for (unsigned i = 0; i < 4; ++i)
        b.push_back((v >> (i * 8)) & 255);
}
} // namespace
std::array<double, 4> EyeFov::renderer_projection() const {
    finite(left);
    finite(right);
    finite(up);
    finite(down);
    if (!(left < 0 && right > 0 && down < 0 && up > 0))
        throw std::invalid_argument("Eye FOV must cross both axes");
    const auto dx = finite(right - left), dy = finite(up - down);
    return {finite(2 / dx), finite(2 / dy), finite(-(right + left) / dx),
            finite(-(up + down) / dy)};
}
EyeFov eye_fov_from_unity_projection(double x, double y, double ox, double oy) {
    finite(x);
    finite(y);
    finite(ox);
    finite(oy);
    if (x <= 0 || y <= 0)
        throw std::invalid_argument("Projection scales must be positive");
    EyeFov f{(-1 + ox) / x, (1 + ox) / x, (1 + oy) / y, (-1 + oy) / y};
    f.renderer_projection();
    return f;
}
ViewState ViewState::from_dict(const Json &j) {
    if (j.at("version") != view_state_version)
        throw std::invalid_argument("Unsupported view-state version");
    ViewState v;
    v.session_id = text(j, "session_id");
    v.reference_space_id = text(j, "reference_space_id");
    v.view_stream_id = text(j, "view_stream_id", false);
    v.view_sequence = integer(j, "view_sequence");
    v.sample_time_ns = integer(j, "sample_time_ns");
    v.predicted_display_time_ns = integer(j, "predicted_display_time_ns");
    v.predicted_display_period_ns = integer(j, "predicted_display_period_ns");
    v.viewport_width = integer(j, "viewport_width", UINT32_MAX);
    v.viewport_height = integer(j, "viewport_height", UINT32_MAX);
    v.refresh_rate_hz = number(j, "refresh_rate_hz");
    if (!v.predicted_display_period_ns || !v.viewport_width || !v.viewport_height ||
        v.refresh_rate_hz < 30 || v.refresh_rate_hz > 240)
        throw std::invalid_argument("Invalid view timing or dimensions");
    const auto &w = j.at("workspace");
    v.workspace = {integer(w, "revision"), vector(w.at("position")), quaternion(w.at("rotation")),
                   number(w, "scale")};
    if (v.workspace.scale <= 0)
        throw std::invalid_argument("Workspace scale must be positive");
    v.left_eye = eye(j.at("left_eye"));
    v.right_eye = eye(j.at("right_eye"));
    return v;
}
ViewState ViewState::from_json(const std::string &value) {
    if (value.size() > maximum_metadata_bytes)
        throw std::invalid_argument("View metadata exceeds 64 KiB");
    return from_dict(Json::parse(value));
}
Json ViewState::to_dict() const {
    return {{"version", view_state_version},
            {"session_id", session_id},
            {"reference_space_id", reference_space_id},
            {"view_stream_id", view_stream_id},
            {"view_sequence", view_sequence},
            {"sample_time_ns", sample_time_ns},
            {"predicted_display_time_ns", predicted_display_time_ns},
            {"predicted_display_period_ns", predicted_display_period_ns},
            {"viewport_width", viewport_width},
            {"viewport_height", viewport_height},
            {"refresh_rate_hz", refresh_rate_hz},
            {"workspace",
             {{"revision", workspace.revision},
              {"position", json(workspace.position)},
              {"rotation", json(workspace.rotation)},
              {"scale", workspace.scale}}},
            {"left_eye", json(left_eye)},
            {"right_eye", json(right_eye)}};
}
Matrix4 eye_view_matrix(const EyeView &e) {
    const auto q = quaternion(json(e.rotation));
    Eigen::Matrix3d r = Eigen::Quaterniond(q.w, q.x, q.y, q.z).toRotationMatrix().transpose();
    Eigen::Matrix4d m = Eigen::Matrix4d::Identity();
    m.block<3, 3>(0, 0) = r;
    m.block<3, 1>(0, 3) =
        -r * Eigen::Vector3d(finite(e.position.x), finite(e.position.y), finite(e.position.z));
    return matrix(m);
}
Matrix4 eye_projection_matrix(const EyeView &e, double n, double f) {
    finite(n);
    finite(f);
    if (n <= 0 || f <= n)
        throw std::invalid_argument("Projection requires 0 < near < far");
    const auto p = e.fov.renderer_projection();
    return {{{p[0], 0, p[2], 0},
             {0, p[1], p[3], 0},
             {0, 0, finite((f + n) / (f - n)), finite(-2 * f * n / (f - n))},
             {0, 0, 1, 0}}};
}
Matrix4 eye_view_projection_matrix(const EyeView &e, double n, double f) {
    const auto p = eye_projection_matrix(e, n, f), v = eye_view_matrix(e);
    Matrix4 m{};
    for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 4; ++c)
            for (int k = 0; k < 4; ++k)
                m[r][c] += p[r][k] * v[k][c];
    return m;
}
Json validate_frame_metadata(const Json &input) {
    if (!input.is_object() || input.at("version") != frame_metadata_version)
        throw std::invalid_argument("Unsupported frame metadata");
    Json j = input;
    static const std::set<std::string> fields{"version",
                                              "session_id",
                                              "frame_id",
                                              "source_view_sequence",
                                              "predicted_display_time_ns",
                                              "workspace_revision",
                                              "render_start_ns",
                                              "render_complete_ns",
                                              "encode_start_ns",
                                              "capture_time_ns",
                                              "stereo_layout",
                                              "width_per_eye",
                                              "height_per_eye",
                                              "codec",
                                              "color_space",
                                              "color_range",
                                              "depth_representation",
                                              "visibility_mask_height",
                                              "frame_marker_height",
                                              "frame_marker",
                                              "video_height",
                                              "depth_near_m",
                                              "depth_far_m",
                                              "view_stream_id"};
    for (const auto &[key, _] : j.items())
        if (!fields.count(key))
            throw std::invalid_argument("Unknown frame metadata field: " + key);
    const Json defaults = {
        {"color_space", "srgb"},       {"color_range", "full"},    {"depth_representation", "none"},
        {"visibility_mask_height", 0}, {"frame_marker_height", 0}, {"frame_marker", 0},
        {"video_height", 0},           {"depth_near_m", 0.05},     {"depth_far_m", 100.0},
        {"view_stream_id", ""}};
    for (const auto &[k, v] : defaults.items())
        if (!j.contains(k))
            j[k] = v;
    text(j, "session_id");
    for (const auto *k :
         {"frame_id", "source_view_sequence", "predicted_display_time_ns", "workspace_revision",
          "render_start_ns", "render_complete_ns", "encode_start_ns", "capture_time_ns"})
        integer(j, k);
    const auto width = integer(j, "width_per_eye", UINT32_MAX),
               height = integer(j, "height_per_eye", UINT32_MAX);
    const auto mask = integer(j, "visibility_mask_height", UINT32_MAX),
               marker = integer(j, "frame_marker_height", UINT32_MAX),
               video = integer(j, "video_height", UINT32_MAX);
    integer(j, "frame_marker", 65535);
    if (!width || !height)
        throw std::invalid_argument("Frame dimensions must be positive");
    const auto layout = text(j, "stereo_layout"), codec = text(j, "codec");
    static const std::set<std::string> layouts{"side_by_side", "side_by_side_with_visibility",
                                               "side_by_side_with_depth", "independent_tracks",
                                               "multiview"};
    if (!layouts.count(layout) || (codec != "png" && codec != "h264"))
        throw std::invalid_argument("Unsupported frame layout or codec");
    if (j.at("color_range") != (codec == "h264" ? "limited" : "full"))
        throw std::invalid_argument("Frame color range disagrees with codec");
    if (codec == "h264" && marker < 8)
        throw std::invalid_argument("H264 requires an in-band frame marker");
    if (layout == "side_by_side_with_visibility" || layout == "side_by_side_with_depth")
        if (!mask || video != height + mask + marker)
            throw std::invalid_argument("Frame video height disagrees with auxiliary layout");
    const auto near = number(j, "depth_near_m"), far = number(j, "depth_far_m");
    if (layout == "side_by_side_with_depth" &&
        (j.at("depth_representation") != "linear_u16_nibbles_in_video" || width % 4 || near <= 0 ||
         far <= near))
        throw std::invalid_argument("Invalid frame depth contract");
    return j;
}
std::vector<std::uint8_t> pack_ros_diagnostic_frame(const Json &metadata,
                                                    std::span<const std::uint8_t> image) {
    const auto text = validate_frame_metadata(metadata).dump();
    if (text.empty() || text.size() > maximum_metadata_bytes || image.empty() ||
        image.size() > maximum_image_bytes)
        throw std::invalid_argument("Diagnostic frame exceeds metadata/image limits");
    std::vector<std::uint8_t> out;
    out.reserve(16 + text.size() + image.size());
    out.insert(out.end(), {'H', 'R', 'V', '3', 1, 0, 0, 0});
    put32(out, text.size());
    put32(out, image.size());
    out.insert(out.end(), text.begin(), text.end());
    out.insert(out.end(), image.begin(), image.end());
    return out;
}
DiagnosticFrameView unpack_ros_diagnostic_frame(std::span<const std::uint8_t> bytes) {
    const std::array<std::uint8_t, 8> prefix{'H', 'R', 'V', '3', 1, 0, 0, 0};
    if (bytes.size() < 16 || !std::equal(prefix.begin(), prefix.end(), bytes.begin()))
        throw std::invalid_argument("Invalid diagnostic frame header");
    const auto m = u32(bytes, 8), i = u32(bytes, 12);
    if (!m || m > maximum_metadata_bytes || !i || i > maximum_image_bytes ||
        bytes.size() != 16 + std::size_t(m) + i)
        throw std::invalid_argument("Invalid diagnostic frame lengths");
    auto metadata =
        validate_frame_metadata(Json::parse(bytes.begin() + 16, bytes.begin() + 16 + m));
    return {std::move(metadata), bytes.subspan(16 + m, i)};
}
double ClockSample::round_trip_ns() const {
    return std::max(0.0, double((static_cast<long double>(client_receive_ns) - client_send_ns) -
                                (static_cast<long double>(server_send_ns) - server_receive_ns)));
}
double ClockSample::server_minus_client_ns() const {
    return double(static_cast<long double>(server_receive_ns) - client_send_ns +
                  static_cast<long double>(server_send_ns) - client_receive_ns) /
           2;
}
ClockAlignment::ClockAlignment(std::size_t n) : maximum_(n) {
    if (n < 3 || n > 65536)
        throw std::invalid_argument("Clock sample bound must be 3..65536");
}
void ClockAlignment::add(const ClockSample &s) {
    if (s.client_send_ns < 0 || s.server_receive_ns < 0 || s.server_send_ns < s.server_receive_ns ||
        s.client_receive_ns < s.client_send_ns)
        throw std::invalid_argument("Invalid clock sample ordering");
    if (samples_.size() == maximum_)
        samples_.pop_front();
    samples_.push_back(s);
}
double ClockAlignment::offset_ns() const {
    std::vector<ClockSample> ranked(samples_.begin(), samples_.end());
    std::stable_sort(ranked.begin(), ranked.end(), [](const auto &a, const auto &b) {
        return a.round_trip_ns() < b.round_trip_ns();
    });
    std::vector<double> values;
    for (std::size_t i = 0; i < std::max<std::size_t>(1, ranked.size() / 3) && i < ranked.size();
         ++i)
        values.push_back(ranked[i].server_minus_client_ns());
    return median(std::move(values));
}
double ClockAlignment::round_trip_ns() const {
    std::vector<double> values;
    for (const auto &s : samples_)
        values.push_back(s.round_trip_ns());
    return median(std::move(values));
}
ViewStreamSequenceGate::ViewStreamSequenceGate(double timeout) : idle_timeout_(finite(timeout)) {
    if (timeout <= 0)
        throw std::invalid_argument("Idle timeout must be positive");
}
std::optional<std::string> ViewStreamSequenceGate::accept(const ViewState &s, double now) {
    finite(now);
    if (now < 0 || (active_ && now < last_received_))
        throw std::invalid_argument("Use a monotonic receipt clock");
    const bool idle = now - last_received_ >= idle_timeout_;
    if (active_) {
        if (s.view_stream_id != *active_) {
            if (std::find(retired_.begin(), retired_.end(), s.view_stream_id) != retired_.end())
                return "retired_view_stream";
            if (!idle)
                return "view_stream_busy";
            if (retired_.size() == 128)
                retired_.pop_front();
            retired_.push_back(*active_);
        } else {
            if (s.sample_time_ns <= last_sample_)
                return "stale_sample";
            if (s.view_sequence <= last_sequence_ && (!s.view_stream_id.empty() || !idle))
                return "stale_sequence";
        }
    }
    active_ = s.view_stream_id;
    last_sequence_ = s.view_sequence;
    last_sample_ = s.sample_time_ns;
    last_received_ = now;
    ++revision_;
    return std::nullopt;
}
ViewState select_newest_view(std::span<const ViewState> states) {
    if (states.empty())
        throw std::invalid_argument("At least one view is required");
    const ViewState *newest = &states[0];
    for (const auto &s : states) {
        if (s.session_id != states[0].session_id)
            throw std::invalid_argument("Mixed view sessions");
        if (s.view_sequence > newest->view_sequence)
            newest = &s;
    }
    return *newest;
}
} // namespace horus::remote_rendering
