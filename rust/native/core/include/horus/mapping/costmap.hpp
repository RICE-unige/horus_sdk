#pragma once
#include <array>
#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <vector>

namespace horus::mapping {
struct GridGeometry {
    std::string frame;
    std::uint32_t width{0}, height{0};
    double resolution{0};
    std::array<double, 7> origin{0,0,0,0,0,0,1};
    void validate() const;
};

// Shared native kernel: borrowed reads, owned snapshots and row-only patches.
class CostmapState {
public:
    bool full(GridGeometry geometry, std::vector<std::int8_t> data, std::uint64_t stamp_ns);
    bool full_raw(GridGeometry geometry, std::span<const std::uint8_t> data, std::uint64_t stamp_ns);
    bool patch(const std::string& frame, std::uint32_t x, std::uint32_t y, std::uint32_t width,
               std::uint32_t height, std::span<const std::int8_t> data, std::uint64_t stamp_ns);
    bool patch_raw(const std::string& frame, std::uint32_t x, std::uint32_t y, std::uint32_t width,
                   std::uint32_t height, std::span<const std::uint8_t> data, std::uint64_t stamp_ns);
    const std::optional<GridGeometry>& geometry() const { return geometry_; }
    std::span<const std::int8_t> data() const { return data_; }
    std::uint64_t stamp_ns() const { return stamp_; }
    std::uint64_t revision() const { return revision_; }
private:
    bool check_patch(const std::string& frame, std::uint32_t x, std::uint32_t y,
                     std::uint32_t width, std::uint32_t height, std::size_t size, std::uint64_t stamp) const;
    std::optional<GridGeometry> geometry_;
    std::vector<std::int8_t> data_;
    std::uint64_t stamp_{0}, revision_{0};
};
std::int8_t raw_to_occupancy(std::uint8_t value);
}
