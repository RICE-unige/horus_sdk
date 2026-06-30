#include "horus/color/color_manager.hpp"

#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <sstream>

namespace horus {
namespace color {

namespace {

// --- Compact MD5 (RFC 1321), used only for the deterministic colour fallback,
// matching Python hashlib.md5 / the Rust md5 crate. ---
struct Md5 {
    std::array<std::uint32_t, 4> state{0x67452301u, 0xefcdab89u, 0x98badcfeu, 0x10325476u};

    static std::uint32_t rotl(std::uint32_t x, std::uint32_t c) {
        return (x << c) | (x >> (32 - c));
    }

    void process_block(const std::uint8_t* block) {
        static const std::array<std::uint32_t, 64> K = [] {
            std::array<std::uint32_t, 64> table{};
            for (int i = 0; i < 64; ++i) {
                table[static_cast<std::size_t>(i)] =
                    static_cast<std::uint32_t>(std::floor(std::abs(std::sin(i + 1.0)) * 4294967296.0));
            }
            return table;
        }();
        static const std::array<std::uint32_t, 64> S{
            7, 12, 17, 22, 7, 12, 17, 22, 7, 12, 17, 22, 7, 12, 17, 22,
            5, 9, 14, 20, 5, 9, 14, 20, 5, 9, 14, 20, 5, 9, 14, 20,
            4, 11, 16, 23, 4, 11, 16, 23, 4, 11, 16, 23, 4, 11, 16, 23,
            6, 10, 15, 21, 6, 10, 15, 21, 6, 10, 15, 21, 6, 10, 15, 21};

        std::array<std::uint32_t, 16> M{};
        for (int i = 0; i < 16; ++i) {
            M[static_cast<std::size_t>(i)] =
                static_cast<std::uint32_t>(block[i * 4]) |
                (static_cast<std::uint32_t>(block[i * 4 + 1]) << 8) |
                (static_cast<std::uint32_t>(block[i * 4 + 2]) << 16) |
                (static_cast<std::uint32_t>(block[i * 4 + 3]) << 24);
        }

        std::uint32_t a = state[0], b = state[1], c = state[2], d = state[3];
        for (std::uint32_t i = 0; i < 64; ++i) {
            std::uint32_t f = 0;
            std::uint32_t g = 0;
            if (i < 16) {
                f = (b & c) | (~b & d);
                g = i;
            } else if (i < 32) {
                f = (d & b) | (~d & c);
                g = (5 * i + 1) % 16;
            } else if (i < 48) {
                f = b ^ c ^ d;
                g = (3 * i + 5) % 16;
            } else {
                f = c ^ (b | ~d);
                g = (7 * i) % 16;
            }
            const std::uint32_t tmp = d;
            d = c;
            c = b;
            b = b + rotl(a + f + K[i] + M[g], S[i]);
            a = tmp;
        }
        state[0] += a;
        state[1] += b;
        state[2] += c;
        state[3] += d;
    }

    std::array<std::uint8_t, 16> digest(const std::string& message) {
        const std::uint64_t bit_len = static_cast<std::uint64_t>(message.size()) * 8;
        std::string padded = message;
        padded.push_back(static_cast<char>(0x80));
        while (padded.size() % 64 != 56) {
            padded.push_back(static_cast<char>(0x00));
        }
        for (int i = 0; i < 8; ++i) {
            padded.push_back(static_cast<char>((bit_len >> (8 * i)) & 0xFF));
        }
        for (std::size_t offset = 0; offset < padded.size(); offset += 64) {
            process_block(reinterpret_cast<const std::uint8_t*>(padded.data() + offset));
        }
        std::array<std::uint8_t, 16> out{};
        for (int i = 0; i < 4; ++i) {
            out[static_cast<std::size_t>(i * 4)] = state[static_cast<std::size_t>(i)] & 0xFF;
            out[static_cast<std::size_t>(i * 4 + 1)] = (state[static_cast<std::size_t>(i)] >> 8) & 0xFF;
            out[static_cast<std::size_t>(i * 4 + 2)] = (state[static_cast<std::size_t>(i)] >> 16) & 0xFF;
            out[static_cast<std::size_t>(i * 4 + 3)] = (state[static_cast<std::size_t>(i)] >> 24) & 0xFF;
        }
        return out;
    }
};

std::uint8_t saturating_add(std::uint8_t value, int delta) {
    const int sum = static_cast<int>(value) + delta;
    return static_cast<std::uint8_t>(sum > 255 ? 255 : (sum < 0 ? 0 : sum));
}

std::vector<std::string> palette_for_scheme(core::ColorScheme scheme) {
    switch (scheme) {
        case core::ColorScheme::BRIGHT:
            return {"#FF0000", "#00FF00", "#0000FF", "#FF7F00", "#FF00FF",
                    "#00FFFF", "#FFFF00", "#FF007F", "#7F00FF", "#00FF7F"};
        case core::ColorScheme::PASTEL:
            return {"#FFB3BA", "#FFDFBA", "#FFFFBA", "#BAFFC9", "#BAE1FF",
                    "#C9BAFF", "#FFBAE1", "#E1BAFF", "#BAFFE1", "#FFE1BA"};
        case core::ColorScheme::DARK:
            return {"#8B0000", "#006400", "#00008B", "#FF8C00", "#8B008B",
                    "#008B8B", "#B8860B", "#8B4513", "#2F4F4F", "#800080"};
        case core::ColorScheme::RAINBOW: {
            std::vector<std::string> palette;
            for (int i = 0; i < 12; ++i) {
                palette.push_back(RGBColor::from_hsv(static_cast<float>(i) * 30.0f, 1.0f, 1.0f, 1.0f).to_hex());
            }
            return palette;
        }
        case core::ColorScheme::NEON: {
            std::vector<std::string> palette;
            for (int i = 0; i < 10; ++i) {
                palette.push_back(RGBColor::from_hsv(static_cast<float>(i) * 36.0f, 1.0f, 1.0f, 1.0f).to_hex());
            }
            return palette;
        }
    }
    return {};
}

}  // namespace

std::string RGBColor::to_hex() const {
    std::ostringstream out;
    out << '#' << std::hex << std::setfill('0') << std::setw(2) << static_cast<int>(r)
        << std::setw(2) << static_cast<int>(g) << std::setw(2) << static_cast<int>(b);
    return out.str();
}

std::array<float, 4> RGBColor::to_normalized_tuple() const {
    return {static_cast<float>(r) / 255.0f, static_cast<float>(g) / 255.0f,
            static_cast<float>(b) / 255.0f, a};
}

std::optional<RGBColor> RGBColor::from_hex(const std::string& hex, float alpha) {
    std::string normalized = hex;
    const auto first = normalized.find_first_not_of(" \t");
    const auto last = normalized.find_last_not_of(" \t");
    if (first == std::string::npos) {
        return std::nullopt;
    }
    normalized = normalized.substr(first, last - first + 1);
    if (!normalized.empty() && normalized[0] == '#') {
        normalized = normalized.substr(1);
    }
    if (normalized.size() != 6) {
        return std::nullopt;
    }
    if (alpha < 0.0f || alpha > 1.0f) {
        return std::nullopt;
    }
    try {
        RGBColor color;
        color.r = static_cast<std::uint8_t>(std::stoi(normalized.substr(0, 2), nullptr, 16));
        color.g = static_cast<std::uint8_t>(std::stoi(normalized.substr(2, 2), nullptr, 16));
        color.b = static_cast<std::uint8_t>(std::stoi(normalized.substr(4, 2), nullptr, 16));
        color.a = alpha;
        return color;
    } catch (...) {
        return std::nullopt;
    }
}

RGBColor RGBColor::from_hsv(float h, float s, float v, float alpha) {
    const float hh = std::fmod(std::fmod(h, 360.0f) + 360.0f, 360.0f);
    const float c = v * s;
    const float x = c * (1.0f - std::fabs(std::fmod(hh / 60.0f, 2.0f) - 1.0f));
    const float m = v - c;
    float r1 = 0, g1 = 0, b1 = 0;
    if (hh < 60.0f) {
        r1 = c; g1 = x; b1 = 0;
    } else if (hh < 120.0f) {
        r1 = x; g1 = c; b1 = 0;
    } else if (hh < 180.0f) {
        r1 = 0; g1 = c; b1 = x;
    } else if (hh < 240.0f) {
        r1 = 0; g1 = x; b1 = c;
    } else if (hh < 300.0f) {
        r1 = x; g1 = 0; b1 = c;
    } else {
        r1 = c; g1 = 0; b1 = x;
    }
    RGBColor color;
    color.r = static_cast<std::uint8_t>(std::lround((r1 + m) * 255.0f));
    color.g = static_cast<std::uint8_t>(std::lround((g1 + m) * 255.0f));
    color.b = static_cast<std::uint8_t>(std::lround((b1 + m) * 255.0f));
    color.a = alpha;
    return color;
}

ColorManager::ColorManager(core::ColorScheme scheme)
    : scheme_(scheme), palette_(palette_for_scheme(scheme)) {}

RGBColor ColorManager::get_robot_color(const std::string& robot_name) {
    const auto it = robot_colors_.find(robot_name);
    if (it != robot_colors_.end()) {
        return it->second;
    }
    const RGBColor color = assign_new_color(robot_name);
    robot_colors_[robot_name] = color;
    return color;
}

RGBColor ColorManager::get_laser_scan_color(const std::string& robot_name, float alpha) {
    const RGBColor base = get_robot_color(robot_name);
    return RGBColor{base.r, base.g, base.b, alpha};
}

RGBColor ColorManager::get_path_color(const std::string& robot_name, const std::string& path_type, float alpha) {
    const RGBColor base = get_robot_color(robot_name);
    if (path_type == "local") {
        return RGBColor{
            saturating_add(base.r, 50),
            saturating_add(base.g, 50),
            saturating_add(base.b, 50),
            std::min(alpha * 0.7f, 1.0f)};
    }
    return RGBColor{base.r, base.g, base.b, alpha};
}

RGBColor ColorManager::get_transform_color(const std::string& robot_name, float alpha) {
    const RGBColor base = get_robot_color(robot_name);
    return RGBColor{base.r, base.g, base.b, alpha};
}

void ColorManager::set_robot_color(const std::string& robot_name, const RGBColor& color) {
    robot_colors_[robot_name] = color;
}

void ColorManager::reset_robot_color(const std::string& robot_name) {
    robot_colors_.erase(robot_name);
}

void ColorManager::clear_all_colors() {
    robot_colors_.clear();
    used_colors_.clear();
    color_index_ = 0;
}

std::map<std::string, std::string> ColorManager::get_color_summary() const {
    std::map<std::string, std::string> summary;
    for (const auto& [robot, color] : robot_colors_) {
        summary[robot] = color.to_hex();
    }
    return summary;
}

RGBColor ColorManager::assign_new_color(const std::string& robot_name) {
    if (color_index_ < palette_.size()) {
        const std::string color_hex = palette_[color_index_];
        ++color_index_;
        used_colors_.push_back(color_hex);
        return RGBColor::from_hex(color_hex, 1.0f).value_or(RGBColor{255, 0, 0, 1.0f});
    }
    return generate_deterministic_color(robot_name);
}

RGBColor ColorManager::generate_deterministic_color(const std::string& robot_name) const {
    Md5 md5;
    const auto digest = md5.digest(robot_name);
    std::uint8_t r = digest[0];
    std::uint8_t g = digest[1];
    std::uint8_t b = digest[2];
    const std::uint8_t min_brightness = 100;
    if (static_cast<int>(r) + static_cast<int>(g) + static_cast<int>(b) <
        static_cast<int>(min_brightness) * 3) {
        r = std::max(r, min_brightness);
        g = std::max(g, min_brightness);
        b = std::max(b, min_brightness);
    }
    return RGBColor{r, g, b, 1.0f};
}

}  // namespace color
}  // namespace horus
