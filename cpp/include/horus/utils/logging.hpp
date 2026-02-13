#ifndef HORUS_UTILS_LOGGING_HPP
#define HORUS_UTILS_LOGGING_HPP

#include <string>

namespace horus {
namespace utils {

void print_step(const std::string& message);
void print_success(const std::string& message);
void print_error(const std::string& message);
void print_info(const std::string& message);

} // namespace utils
} // namespace horus

#endif // HORUS_UTILS_LOGGING_HPP

