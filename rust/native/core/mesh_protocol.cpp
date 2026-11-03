#include "horus/mapping/mesh_protocol.hpp"
#include <bit>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>

namespace horus::mapping {
namespace {
template<class T> void append(std::uint8_t*& out, T value) {
    if constexpr (std::endian::native == std::endian::little) std::memcpy(out, &value, sizeof(T));
    else for (std::size_t i=0; i<sizeof(T); ++i) out[i] = static_cast<std::uint8_t>(value >> (8*i));
    out += sizeof(T);
}
template<class T> T read(std::span<const std::uint8_t> data, std::size_t offset) {
    T value = 0;
    for (std::size_t i=0; i<sizeof(T); ++i) value |= T(data[offset+i]) << (8*i);
    return value;
}
}
std::vector<std::uint8_t> encode_live_payload(std::uint32_t chunk, std::array<std::int32_t,3> page,
    std::uint64_t epoch, std::uint64_t revision, MeshOperation operation,
    const BlockGeometry* g, std::size_t budget) {
    const std::uint64_t vertices = g ? g->vertices.size() : 0, indices = g ? g->triangles.size() : 0;
    const bool color = g && g->colors.size() == vertices;
    if (static_cast<unsigned>(operation) > 2 || vertices > UINT32_MAX || indices > UINT32_MAX)
        throw std::invalid_argument("Invalid HMSH operation/counts");
    if (g) {
        if (operation != MeshOperation::Upsert || indices%3 || (!g->colors.empty() && !color))
            throw std::invalid_argument("Invalid HMSH geometry");
        for (const auto& vertex : g->vertices) for (auto v : vertex)
            if (!std::isfinite(v)) throw std::invalid_argument("Non-finite HMSH vertex");
        for (auto i : g->triangles) if (i >= vertices) throw std::invalid_argument("HMSH index outside vertices");
    }
    const auto size = HMSH_V2_HEADER_SIZE + vertices*(color ? 16 : 12) + indices*4;
    if (size > budget) throw std::invalid_argument("HMSH page exceeds budget; split pages without reducing quality");
    // The validated exact length permits contiguous writes without a capacity
    // branch per byte; memcpy handles unaligned wire storage without aliasing UB.
    std::vector<std::uint8_t> out(static_cast<std::size_t>(size));
    auto* cursor = out.data();
    append(cursor,std::uint32_t(0x484d5348)); append(cursor,std::uint32_t(2)); append(cursor,chunk);
    for (auto key : page) append(cursor,std::bit_cast<std::uint32_t>(key));
    append(cursor,epoch); append(cursor,revision);
    *cursor++ = static_cast<std::uint8_t>(operation);
    *cursor++ = static_cast<std::uint8_t>(color);
    append(cursor,std::uint16_t(0));
    append(cursor,static_cast<std::uint32_t>(vertices)); append(cursor,static_cast<std::uint32_t>(indices));
    if (g) {
        for (std::size_t i=0; i<vertices; ++i) {
            for (auto value : g->vertices[i]) append(cursor,std::bit_cast<std::uint32_t>(value));
            if (color) { std::memcpy(cursor,g->colors[i].data(),4); cursor+=4; }
        }
        for (auto index : g->triangles) append(cursor,index);
    }
    return out;
}
LiveHeader decode_live_header(std::span<const std::uint8_t> bytes) {
    if (bytes.size() < HMSH_V2_HEADER_SIZE) throw std::invalid_argument("Truncated HMSH header");
    if (read<std::uint32_t>(bytes,0) != 0x484d5348 || read<std::uint32_t>(bytes,4) != 2)
        throw std::invalid_argument("Not HMSH v2");
    if (bytes[40]>2 || bytes[41]>1 || bytes[42] || bytes[43]) throw std::invalid_argument("Invalid HMSH flags");
    return {read<std::uint32_t>(bytes,8),
        {std::bit_cast<std::int32_t>(read<std::uint32_t>(bytes,12)),std::bit_cast<std::int32_t>(read<std::uint32_t>(bytes,16)),std::bit_cast<std::int32_t>(read<std::uint32_t>(bytes,20))},
        read<std::uint64_t>(bytes,24),read<std::uint64_t>(bytes,32),static_cast<MeshOperation>(bytes[40]),bytes[41]!=0,
        read<std::uint32_t>(bytes,44),read<std::uint32_t>(bytes,48)};
}
}
