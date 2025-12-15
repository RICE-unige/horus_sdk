#ifndef HORUS_UTILS_BRANDING_HPP
#define HORUS_UTILS_BRANDING_HPP

#include <string>

namespace horus {
namespace utils {

/**
 * @brief HORUS SDK version
 */
constexpr const char* VERSION = "0.1.0";

/**
 * @brief Display HORUS ASCII art header with version information
 */
void show_ascii_art();

/**
 * @brief Legacy function name for backward compatibility
 */
inline void print_header() { show_ascii_art(); }

/**
 * @brief Get HORUS SDK version string
 * @return Version string
 */
inline std::string get_version() {
    return VERSION;
}

} // namespace utils
} // namespace horus

#endif // HORUS_UTILS_BRANDING_HPP
