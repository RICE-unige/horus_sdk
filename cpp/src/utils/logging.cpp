#include "horus/utils/logging.hpp"

#include <iostream>

namespace horus {
namespace utils {

void print_step(const std::string& message) { std::cout << "* " << message << std::endl; }
void print_success(const std::string& message) { std::cout << "+ " << message << std::endl; }
void print_error(const std::string& message) { std::cerr << "- " << message << std::endl; }
void print_info(const std::string& message) { std::cout << message << std::endl; }

} // namespace utils
} // namespace horus

