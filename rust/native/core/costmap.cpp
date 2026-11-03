#include "horus/mapping/costmap.hpp"
#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <stdexcept>

namespace horus::mapping {
namespace {
constexpr std::int8_t convert_raw(std::uint8_t value) {
    if (value == 0)
        return 0;
    if (value == 253)
        return 99;
    if (value == 254)
        return 100;
    if (value == 255)
        return -1;
    return static_cast<std::int8_t>(1 + 97 * (value - 1) / 251);
}
constexpr auto raw_cost_table = [] {
    std::array<std::int8_t, 256> table{};
    for (std::size_t i = 0; i < table.size(); ++i)
        table[i] = convert_raw(static_cast<std::uint8_t>(i));
    return table;
}();
std::string frame_name(const std::string &frame) {
    auto first = frame.find_first_not_of(" \t\n\r");
    auto last = frame.find_last_not_of(" \t\n\r");
    if (first == std::string::npos)
        return {};
    auto value = frame.substr(first, last - first + 1);
    first = value.find_first_not_of('/');
    return first == std::string::npos ? std::string{} : value.substr(first);
}
void validate_cells(std::span<const std::int8_t> cells) {
    if (std::any_of(cells.begin(), cells.end(), [](auto c) { return c < -1 || c > 100; }))
        throw std::invalid_argument("Occupancy cells must be -1..100");
}
template <class T> bool aliases(std::span<const T> input, const std::vector<std::int8_t> &owned) {
    const std::less<const void *> less;
    return !input.empty() && !owned.empty() && less(input.data(), owned.data() + owned.size()) &&
           less(owned.data(), input.data() + input.size());
}
std::vector<std::int8_t> decode(std::span<const std::uint8_t> raw) {
    std::vector<std::int8_t> out(raw.size());
    std::transform(raw.begin(), raw.end(), out.begin(),
                   [](auto value) { return raw_cost_table[value]; });
    return out;
}
} // namespace
std::int8_t raw_to_occupancy(std::uint8_t value) { return convert_raw(value); }
void GridGeometry::validate() const {
    if (frame_name(frame).empty())
        throw std::invalid_argument("Costmap frame is empty");
    if (!width || !height || width > 4096 || height > 4096 ||
        std::uint64_t(width) * height > 4 * 1024 * 1024)
        throw std::invalid_argument("Costmap capacity is 4096 per side, 4M cells");
    if (!std::isfinite(resolution) || resolution <= 0)
        throw std::invalid_argument("Invalid costmap resolution");
    for (auto v : origin)
        if (!std::isfinite(v))
            throw std::invalid_argument("Invalid costmap origin");
    const auto norm =
        std::hypot(std::hypot(origin[3], origin[4]), std::hypot(origin[5], origin[6]));
    if (std::abs(norm - 1) > 1e-3)
        throw std::invalid_argument("Origin quaternion is not normalized");
    if (std::max(width, height) * resolution +
            std::max({std::abs(origin[0]), std::abs(origin[1]), std::abs(origin[2])}) >
        std::numeric_limits<float>::max())
        throw std::invalid_argument("Costmap geometry exceeds headset coordinate range");
}
bool CostmapState::full(GridGeometry geometry, std::vector<std::int8_t> data, std::uint64_t stamp) {
    geometry.validate();
    if (data.size() != std::uint64_t(geometry.width) * geometry.height)
        throw std::invalid_argument("Costmap payload length mismatch");
    validate_cells(data);
    if (geometry_ && stamp < stamp_)
        return false;
    geometry_ = std::move(geometry);
    data_ = std::move(data);
    stamp_ = stamp;
    ++revision_;
    return true;
}
bool CostmapState::full_raw(GridGeometry geometry, std::span<const std::uint8_t> data,
                            std::uint64_t stamp) {
    geometry.validate();
    if (data.size() != std::uint64_t(geometry.width) * geometry.height)
        throw std::invalid_argument("Costmap payload length mismatch");
    return full(std::move(geometry), decode(data), stamp);
}
bool CostmapState::check_patch(const std::string &frame, std::uint32_t x, std::uint32_t y,
                               std::uint32_t w, std::uint32_t h, std::size_t size,
                               std::uint64_t stamp) const {
    if (!geometry_)
        throw std::invalid_argument("Costmap snapshot required before patches");
    if (stamp < stamp_)
        return false;
    if (frame_name(frame) != frame_name(geometry_->frame))
        throw std::invalid_argument("Costmap patch frame differs");
    if (!w || !h || std::uint64_t(x) + w > geometry_->width ||
        std::uint64_t(y) + h > geometry_->height)
        throw std::invalid_argument("Costmap patch outside snapshot");
    if (size != std::uint64_t(w) * h)
        throw std::invalid_argument("Costmap patch payload length mismatch");
    return true;
}
bool CostmapState::patch(const std::string &frame, std::uint32_t x, std::uint32_t y,
                         std::uint32_t w, std::uint32_t h, std::span<const std::int8_t> data,
                         std::uint64_t stamp) {
    if (!check_patch(frame, x, y, w, h, data.size(), stamp))
        return false;
    validate_cells(data);
    std::vector<std::int8_t> staging;
    if (aliases(data, data_)) {
        staging.assign(data.begin(), data.end());
        data = staging;
    }
    for (std::size_t row = 0; row < h; ++row)
        std::copy_n(data.begin() + row * w, w, data_.begin() + (y + row) * geometry_->width + x);
    stamp_ = stamp;
    ++revision_;
    return true;
}
bool CostmapState::patch_raw(const std::string &frame, std::uint32_t x, std::uint32_t y,
                             std::uint32_t w, std::uint32_t h, std::span<const std::uint8_t> data,
                             std::uint64_t stamp) {
    if (!check_patch(frame, x, y, w, h, data.size(), stamp))
        return false;
    std::vector<std::uint8_t> staging;
    if (aliases(data, data_)) {
        staging.assign(data.begin(), data.end());
        data = staging;
    }
    for (std::size_t row = 0; row < h; ++row)
        std::transform(data.begin() + row * w, data.begin() + (row + 1) * w,
                       data_.begin() + (y + row) * geometry_->width + x,
                       [](auto value) { return raw_cost_table[value]; });
    stamp_ = stamp;
    ++revision_;
    return true;
}
} // namespace horus::mapping
