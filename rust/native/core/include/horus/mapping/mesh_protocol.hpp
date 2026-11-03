#pragma once
#include <array>
#include <cstdint>
#include <span>
#include <vector>

namespace horus::mapping {
enum class MeshOperation : std::uint8_t { Upsert=0, Delete=1, Clear=2 };
struct BlockGeometry {
    std::vector<std::array<float,3>> vertices;
    std::vector<std::array<std::uint8_t,4>> colors;
    std::vector<std::uint32_t> triangles;
};
struct LiveHeader {
    std::uint32_t chunk_id;
    std::array<std::int32_t,3> page_key;
    std::uint64_t epoch, revision;
    MeshOperation operation;
    bool has_color;
    std::uint32_t vertex_count, index_count;
};
inline constexpr std::size_t HMSH_V2_HEADER_SIZE = 52;
std::vector<std::uint8_t> encode_live_payload(std::uint32_t chunk_id, std::array<std::int32_t,3> page_key,
    std::uint64_t epoch, std::uint64_t revision, MeshOperation operation,
    const BlockGeometry* geometry, std::size_t max_payload_bytes);
LiveHeader decode_live_header(std::span<const std::uint8_t> payload);
}
