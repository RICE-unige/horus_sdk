#include "horus/utils/branding.hpp"
#include <iostream>

namespace horus {
namespace utils {

void show_ascii_art() {
    const char* c_h = "\033[96m";
    const char* c_o = "\033[94m";
    const char* c_r = "\033[95m";
    const char* c_u = "\033[35m";
    const char* c_s = "\033[91m";
    const char* c_dim = "\033[2m";
    const char* c_reset = "\033[0m";

    std::cout << std::endl;

    std::cout << "    " << c_h << "\u2588\u2588\u2557  \u2588\u2588\u2557 " << c_reset
              << c_o << "\u2588\u2588\u2588\u2588\u2588\u2588\u2557 " << c_reset
              << c_r << "\u2588\u2588\u2588\u2588\u2588\u2588\u2557 " << c_reset
              << c_u << "\u2588\u2588\u2557   \u2588\u2588\u2557" << c_reset
              << c_s << "\u2588\u2588\u2588\u2588\u2588\u2588\u2588\u2557" << c_reset << std::endl;

    std::cout << "    " << c_h << "\u2588\u2588\u2551  \u2588\u2588\u2551" << c_reset
              << c_o << "\u2588\u2588\u2554\u2550\u2550\u2550\u2588\u2588\u2557" << c_reset
              << c_r << "\u2588\u2588\u2554\u2550\u2550\u2588\u2588\u2557" << c_reset
              << c_u << "\u2588\u2588\u2551   \u2588\u2588\u2551" << c_reset
              << c_s << "\u2588\u2588\u2554\u2550\u2550\u2550\u2550\u255d" << c_reset << std::endl;

    std::cout << "    " << c_h << "\u2588\u2588\u2588\u2588\u2588\u2588\u2588\u2551" << c_reset
              << c_o << "\u2588\u2588\u2551   \u2588\u2588\u2551" << c_reset
              << c_r << "\u2588\u2588\u2588\u2588\u2588\u2588\u2554\u255d" << c_reset
              << c_u << "\u2588\u2588\u2551   \u2588\u2588\u2551" << c_reset
              << c_s << "\u2588\u2588\u2588\u2588\u2588\u2588\u2588\u2557" << c_reset << std::endl;

    std::cout << "    " << c_h << "\u2588\u2588\u2554\u2550\u2550\u2588\u2588\u2551" << c_reset
              << c_o << "\u2588\u2588\u2551   \u2588\u2588\u2551" << c_reset
              << c_r << "\u2588\u2588\u2554\u2550\u2550\u2588\u2588\u2557" << c_reset
              << c_u << "\u2588\u2588\u2551   \u2588\u2588\u2551" << c_reset
              << c_s << "\u255a\u2550\u2550\u2550\u2550\u2588\u2588\u2551" << c_reset << std::endl;

    std::cout << "    " << c_h << "\u2588\u2588\u2551  \u2588\u2588\u2551" << c_reset
              << c_o << "\u255a\u2588\u2588\u2588\u2588\u2588\u2588\u2554\u255d" << c_reset
              << c_r << "\u2588\u2588\u2551  \u2588\u2588\u2551" << c_reset
              << c_u << "\u255a\u2588\u2588\u2588\u2588\u2588\u2588\u2554\u255d" << c_reset
              << c_s << "\u2588\u2588\u2588\u2588\u2588\u2588\u2588\u2551" << c_reset << std::endl;

    std::cout << "    " << c_h << "\u255a\u2550\u255d  \u255a\u2550\u255d " << c_reset
              << c_o << "\u255a\u2550\u2550\u2550\u2550\u2550\u255d " << c_reset
              << c_r << "\u255a\u2550\u255d  \u255a\u2550\u255d " << c_reset
              << c_u << "\u255a\u2550\u2550\u2550\u2550\u2550\u255d " << c_reset
              << c_s << "\u255a\u2550\u2550\u2550\u2550\u2550\u2550\u255d" << c_reset << std::endl;

    std::cout << std::endl;
    std::cout << "    " << c_o << "        Holistic Operational Reality" << c_reset << std::endl;
    std::cout << "    " << c_r << "            for Unified Systems" << c_reset << std::endl;
    std::cout << std::endl;
    std::cout << "    " << c_dim << "\xF0\x9F\xA4\x96 Mixed Reality Robot Management Platform" << c_reset << std::endl;
    std::cout << "    " << c_dim << "\xF0\x9F\x8F\x97\xEF\xB8\x8F Developed at RICE Lab, University of Genoa" << c_reset << std::endl;
    std::cout << "    " << c_dim << "HORUS SDK v" << VERSION << c_reset << std::endl;
    std::cout << std::endl;

    std::cout << "============================================================" << std::endl;
}

} // namespace utils
} // namespace horus
