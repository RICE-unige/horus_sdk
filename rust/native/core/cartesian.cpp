#include "horus/teaching/cartesian.hpp"
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <set>

namespace horus::teaching {
namespace {
Eigen::Vector3d vector(const Json &v) {
    return {v.at(0).get<double>(), v.at(1).get<double>(), v.at(2).get<double>()};
}
Eigen::Quaterniond rotation(const Json &v) {
    return Eigen::Quaterniond(v.at(3).get<double>(), v.at(0).get<double>(), v.at(1).get<double>(),
                              v.at(2).get<double>())
        .normalized();
}
} // namespace
struct CartesianAdapter::Impl {
    AdapterCapabilities cap;
    CartesianLimits limits;
    std::function<Json()> feedback;
    std::function<Json(const Json &)> publish;
    std::function<bool(const Json &)> hold;
    std::function<double()> clock;
    std::vector<std::string> hands;
    Json offsets, authority, previous, measured;
    std::map<std::string, std::pair<Eigen::Vector3d, Eigen::Quaterniond>> anchors;
    double previous_time = 0;
    Eigen::Quaterniond frame;
    Json read() const {
        auto value = feedback();
        double age = clock() - value.at("received_monotonic").get<double>();
        if (value.value("ready", Json()) != true || !std::isfinite(age) || age < 0 ||
            age > limits.feedback_timeout_s)
            throw std::runtime_error("Robot feedback is stale or controller is not ready");
        for (auto &hand : hands)
            validate_pose(value.at("poses").at(hand));
        return value;
    }
    std::pair<Eigen::Vector3d, Eigen::Quaterniond> tool(const Json &pose,
                                                        const std::string &hand) const {
        validate_pose(pose);
        auto p = vector(pose.at("position"));
        auto r = rotation(pose.at("rotation"));
        if (offsets.contains(hand)) {
            p += r * vector(offsets.at(hand).at("position"));
            r = r * rotation(offsets.at(hand).at("rotation"));
        }
        return {p, r};
    }
};
CartesianAdapter::CartesianAdapter(AdapterCapabilities cap, CartesianLimits limits,
                                   std::function<Json()> feedback,
                                   std::function<Json(const Json &)> publish,
                                   std::function<bool(const Json &)> hold,
                                   std::vector<std::string> hands, Json offsets,
                                   std::function<double()> clock)
    : impl_(std::make_unique<Impl>()) {
    validate_pose(
        Json{{"position", limits.minimum}, {"rotation", limits.floor_to_command_rotation}});
    validate_pose(
        Json{{"position", limits.maximum}, {"rotation", limits.floor_to_command_rotation}});
    for (int i = 0; i < 3; ++i)
        if (limits.minimum[i] >= limits.maximum[i])
            throw std::invalid_argument("Invalid Cartesian bounds");
    for (auto v : {limits.max_speed_m_s, limits.max_rotation_rad_s, limits.feedback_timeout_s,
                   limits.translation_scale})
        if (!std::isfinite(v) || v <= 0)
            throw std::invalid_argument("Invalid Cartesian limit");
    if (cap.action_representation != "cartesian_pose_absolute" ||
        cap.authority_contract == "unavailable")
        throw std::invalid_argument("Declare Cartesian action and authority contracts");
    if (cap.command_boundary != "published" && cap.command_boundary != "accepted" &&
        cap.command_boundary != "scheduled")
        throw std::invalid_argument("Declare an observed controller boundary");
    std::set<std::string> unique(hands.begin(), hands.end());
    if (hands.empty() || unique.size() != hands.size() ||
        std::any_of(hands.begin(), hands.end(),
                    [](auto &h) { return h != "left" && h != "right"; }))
        throw std::invalid_argument("Invalid hands");
    if (!offsets.is_object() || !feedback || !publish || !hold || !clock)
        throw std::invalid_argument("Missing callbacks/offsets");
    for (auto it = offsets.begin(); it != offsets.end(); ++it) {
        if (!unique.count(it.key()))
            throw std::invalid_argument("Offset for uncontrolled hand");
        validate_pose(it.value());
    }
    impl_->cap = std::move(cap);
    impl_->limits = limits;
    impl_->feedback = std::move(feedback);
    impl_->publish = std::move(publish);
    impl_->hold = std::move(hold);
    impl_->hands = std::move(hands);
    impl_->offsets = std::move(offsets);
    impl_->clock = std::move(clock);
    impl_->frame = rotation(Json(limits.floor_to_command_rotation));
}
CartesianAdapter::~CartesianAdapter() = default;
AdapterCapabilities CartesianAdapter::capabilities() const { return impl_->cap; }
std::pair<bool, std::string> CartesianAdapter::ready() {
    try {
        impl_->read();
        return {true, ""};
    } catch (const std::exception &e) {
        return {false, e.what()};
    }
}
void CartesianAdapter::anchor(const Json &sample) {
    auto authority = sample.at("authority");
    for (auto key : {"robot_name", "owner", "session_id", "authority_generation"})
        if (!authority.contains(key) || !authority[key].is_string() || authority[key] == "")
            throw std::invalid_argument("Coordinator-assigned authority required");
    auto measured = impl_->read().at("poses");
    std::map<std::string, std::pair<Eigen::Vector3d, Eigen::Quaterniond>> anchors;
    for (auto &hand : impl_->hands)
        anchors.emplace(hand, impl_->tool(sample.at(hand), hand));
    impl_->anchors = std::move(anchors);
    impl_->measured = measured;
    impl_->previous = measured;
    impl_->authority = authority;
    impl_->previous_time = impl_->clock();
}
Json CartesianAdapter::apply(const Json &sample) {
    impl_->read();
    if (impl_->anchors.empty())
        throw std::runtime_error("Anchor before sending targets");
    if (sample.at("authority") != impl_->authority)
        throw std::runtime_error("Motion authority changed; hold and re-anchor");
    double now = impl_->clock(),
           dt = std::min(now - impl_->previous_time, impl_->limits.feedback_timeout_s);
    if (!std::isfinite(dt) || dt <= 0)
        throw std::runtime_error("Invalid control interval");
    Json targets = Json::object();
    for (auto &hand : impl_->hands) {
        const auto &[anchor_p, anchor_r] = impl_->anchors.at(hand);
        const auto &measured = impl_->measured.at(hand);
        const auto [p, r] = impl_->tool(sample.at(hand), hand);
        Eigen::Vector3d target = vector(measured.at("position")) +
                                 impl_->frame * (p - anchor_p) * impl_->limits.translation_scale;
        Eigen::Quaterniond target_r = impl_->frame * r * anchor_r.conjugate() *
                                      impl_->frame.conjugate() * rotation(measured.at("rotation"));
        target_r.normalize();
        for (int i = 0; i < 3; ++i)
            if (target[i] < impl_->limits.minimum[i] || target[i] > impl_->limits.maximum[i])
                throw std::runtime_error("Cartesian workspace limit reached");
        double delta = (target - vector(impl_->previous.at(hand).at("position"))).norm();
        double angle = target_r.angularDistance(rotation(impl_->previous.at(hand).at("rotation")));
        if (delta > impl_->limits.max_speed_m_s * dt ||
            angle > impl_->limits.max_rotation_rad_s * dt)
            throw std::runtime_error("Cartesian speed limit reached; hold and re-anchor");
        targets[hand] = {{"position", {target.x(), target.y(), target.z()}},
                         {"rotation", {target_r.x(), target_r.y(), target_r.z(), target_r.w()}}};
    }
    Json bundle = {{"sequence", sample.at("sequence")},
                   {"frame_id", impl_->cap.command_frame},
                   {"targets", targets},
                   {"tools", sample.value("tools", Json::object())},
                   {"representation", "cartesian_pose_absolute"},
                   {"authority", impl_->authority}};
    auto receipt = impl_->publish(bundle);
    if (!receipt.is_object() || receipt.value("ok", Json()) != true ||
        receipt.value("boundary", "") != impl_->cap.command_boundary)
        throw std::runtime_error("Controller receipt does not match the declared action boundary");
    impl_->previous = targets;
    impl_->previous_time = now;
    bundle["receipt"] = receipt;
    bundle["boundary"] = impl_->cap.command_boundary;
    return bundle;
}
bool CartesianAdapter::hold(const Json &context) {
    impl_->anchors.clear();
    impl_->authority = nullptr;
    return impl_->hold(context);
}
} // namespace horus::teaching
