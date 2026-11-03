#pragma once
#include "session.hpp"
#include <array>

namespace horus::teaching {
struct CartesianLimits {
    std::array<double, 3> minimum, maximum;
    double max_speed_m_s, max_rotation_rad_s;
    double feedback_timeout_s = 0.2, translation_scale = 1.0;
    std::array<double, 4> floor_to_command_rotation{0, 0, 0, 1};
};
class CartesianAdapter final : public TeachAdapter {
  public:
    CartesianAdapter(AdapterCapabilities, CartesianLimits, std::function<Json()> feedback,
                     std::function<Json(const Json &)> publish,
                     std::function<bool(const Json &)> hold,
                     std::vector<std::string> hands = {"left", "right"},
                     Json tool_offsets = Json::object(),
                     std::function<double()> clock = monotonic_seconds);
    ~CartesianAdapter();
    AdapterCapabilities capabilities() const override;
    std::pair<bool, std::string> ready() override;
    void anchor(const Json &) override;
    Json apply(const Json &) override;
    bool hold(const Json &) override;

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
} // namespace horus::teaching
