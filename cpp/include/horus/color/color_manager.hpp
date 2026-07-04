#ifndef HORUS_COLOR_COLOR_MANAGER_HPP
#define HORUS_COLOR_COLOR_MANAGER_HPP

#include "horus/core/types.hpp"

#include <array>
#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <vector>

namespace horus {
namespace color {

struct RGBColor {
    std::uint8_t r{0};
    std::uint8_t g{0};
    std::uint8_t b{0};
    float a{1.0f};

    std::string to_hex() const;
    std::array<float, 4> to_normalized_tuple() const;

    static std::optional<RGBColor> from_hex(const std::string& hex, float alpha = 1.0f);
    static RGBColor from_hsv(float h, float s, float v, float alpha = 1.0f);

    bool operator==(const RGBColor& other) const {
        return r == other.r && g == other.g && b == other.b && a == other.a;
    }
};

// Deterministic per-robot colour assignment, matching the Python and Rust SDKs:
// a fixed palette per scheme, then an MD5-derived fallback once the palette is
// exhausted.
class ColorManager {
public:
    explicit ColorManager(core::ColorScheme scheme = core::ColorScheme::BRIGHT);

    core::ColorScheme color_scheme() const { return scheme_; }

    RGBColor get_robot_color(const std::string& robot_name);
    RGBColor get_laser_scan_color(const std::string& robot_name, float alpha);
    RGBColor get_path_color(const std::string& robot_name, const std::string& path_type, float alpha);
    RGBColor get_transform_color(const std::string& robot_name, float alpha);

    void set_robot_color(const std::string& robot_name, const RGBColor& color);
    void reset_robot_color(const std::string& robot_name);
    void clear_all_colors();

    std::map<std::string, RGBColor> get_all_robot_colors() const { return robot_colors_; }
    std::map<std::string, std::string> get_color_summary() const;

private:
    RGBColor assign_new_color(const std::string& robot_name);
    RGBColor generate_deterministic_color(const std::string& robot_name) const;

    core::ColorScheme scheme_;
    std::map<std::string, RGBColor> robot_colors_;
    std::vector<std::string> used_colors_;
    std::size_t color_index_{0};
    std::vector<std::string> palette_;
};

}  // namespace color
}  // namespace horus

#endif  // HORUS_COLOR_COLOR_MANAGER_HPP
