#include "horus/color/color_manager.hpp"

#ifdef NDEBUG
#undef NDEBUG
#endif
#include <algorithm>
#include <cassert>
#include <iostream>
#include <string>

using horus::color::ColorManager;
using horus::color::RGBColor;
using horus::core::ColorScheme;

int main() {
    // Palette assignment + caching.
    ColorManager bright(ColorScheme::BRIGHT);
    const auto a = bright.get_robot_color("a");
    assert(a.r == 255 && a.g == 0 && a.b == 0);
    assert(a.to_hex() == "#ff0000");
    assert(bright.get_robot_color("a") == a);
    const auto b = bright.get_robot_color("b");
    assert(b.r == 0 && b.g == 255 && b.b == 0);

    // HSV conversion.
    const auto red = RGBColor::from_hsv(0.0f, 1.0f, 1.0f);
    assert(red.r == 255 && red.g == 0 && red.b == 0);
    const auto green = RGBColor::from_hsv(120.0f, 1.0f, 1.0f);
    assert(green.r == 0 && green.g == 255 && green.b == 0);

    // Hex parsing.
    const auto orange = RGBColor::from_hex("#FF7F00").value();
    assert(orange.r == 255 && orange.g == 127 && orange.b == 0);
    assert(!RGBColor::from_hex("nothex").has_value());

    // MD5 deterministic fallback parity: exhaust the 10-colour palette, then the
    // next robot is coloured from md5(name). md5("abc") = 90 01 50 ... and the
    // min-brightness lift makes it (144, 100, 100), matching Python/Rust.
    ColorManager fallback(ColorScheme::BRIGHT);
    for (int i = 0; i < 10; ++i) {
        fallback.get_robot_color("robot_" + std::to_string(i));
    }
    const auto det = fallback.get_robot_color("abc");
    assert(det.r == 144 && det.g == 100 && det.b == 100);
    assert(fallback.get_robot_color("abc") == det);

    // Path colour: local brightens by 50 (saturating).
    ColorManager paths(ColorScheme::BRIGHT);
    const auto local = paths.get_path_color("x", "local", 1.0f);
    const auto base = paths.get_robot_color("x");
    assert(local.r == std::min(255, static_cast<int>(base.r) + 50));

    // Other schemes produce valid palettes.
    assert(ColorManager(ColorScheme::RAINBOW).get_robot_color("r").to_hex().size() == 7);
    assert(ColorManager(ColorScheme::NEON).get_robot_color("n").to_hex().size() == 7);

    std::cout << "cpp_color_tests passed" << std::endl;
    return 0;
}
