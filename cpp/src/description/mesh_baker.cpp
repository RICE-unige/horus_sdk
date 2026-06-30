#include "horus/description/mesh_baker.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <limits>
#include <map>
#include <regex>
#include <sstream>

namespace horus {
namespace description {

namespace {

constexpr char kBase64Alphabet[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

struct Triangle {
    std::array<float, 3> normal{{0.0f, 0.0f, 0.0f}};
    std::array<std::array<float, 3>, 3> verts{};
};

std::uint32_t f32_bits(float value) {
    std::uint32_t bits = 0;
    std::memcpy(&bits, &value, 4);
    return bits;
}

float read_f32_le(const std::uint8_t* p) {
    const std::uint32_t bits = static_cast<std::uint32_t>(p[0]) |
                               (static_cast<std::uint32_t>(p[1]) << 8) |
                               (static_cast<std::uint32_t>(p[2]) << 16) |
                               (static_cast<std::uint32_t>(p[3]) << 24);
    float value = 0.0f;
    std::memcpy(&value, &bits, 4);
    return value;
}

void push_f32_le(std::vector<std::uint8_t>& out, float value) {
    const std::uint32_t bits = f32_bits(value);
    out.push_back(static_cast<std::uint8_t>(bits & 0xFF));
    out.push_back(static_cast<std::uint8_t>((bits >> 8) & 0xFF));
    out.push_back(static_cast<std::uint8_t>((bits >> 16) & 0xFF));
    out.push_back(static_cast<std::uint8_t>((bits >> 24) & 0xFF));
}

void push_i32_le(std::vector<std::uint8_t>& out, std::int32_t value) {
    const std::uint32_t bits = static_cast<std::uint32_t>(value);
    out.push_back(static_cast<std::uint8_t>(bits & 0xFF));
    out.push_back(static_cast<std::uint8_t>((bits >> 8) & 0xFF));
    out.push_back(static_cast<std::uint8_t>((bits >> 16) & 0xFF));
    out.push_back(static_cast<std::uint8_t>((bits >> 24) & 0xFF));
}

bool is_binary_stl(const std::vector<std::uint8_t>& bytes) {
    if (bytes.size() < 84) {
        return false;
    }
    const std::uint32_t count = static_cast<std::uint32_t>(bytes[80]) |
                                (static_cast<std::uint32_t>(bytes[81]) << 8) |
                                (static_cast<std::uint32_t>(bytes[82]) << 16) |
                                (static_cast<std::uint32_t>(bytes[83]) << 24);
    return bytes.size() == 84 + static_cast<std::size_t>(count) * 50;
}

std::vector<Triangle> parse_binary_stl(const std::vector<std::uint8_t>& bytes) {
    const std::uint32_t count = static_cast<std::uint32_t>(bytes[80]) |
                                (static_cast<std::uint32_t>(bytes[81]) << 8) |
                                (static_cast<std::uint32_t>(bytes[82]) << 16) |
                                (static_cast<std::uint32_t>(bytes[83]) << 24);
    std::vector<Triangle> triangles;
    triangles.reserve(count);
    std::size_t offset = 84;
    const std::uint8_t* data = bytes.data();
    for (std::uint32_t i = 0; i < count; ++i) {
        Triangle tri;
        tri.normal = {read_f32_le(data + offset), read_f32_le(data + offset + 4), read_f32_le(data + offset + 8)};
        tri.verts[0] = {read_f32_le(data + offset + 12), read_f32_le(data + offset + 16), read_f32_le(data + offset + 20)};
        tri.verts[1] = {read_f32_le(data + offset + 24), read_f32_le(data + offset + 28), read_f32_le(data + offset + 32)};
        tri.verts[2] = {read_f32_le(data + offset + 36), read_f32_le(data + offset + 40), read_f32_le(data + offset + 44)};
        triangles.push_back(tri);
        offset += 50;
    }
    return triangles;
}

std::vector<Triangle> parse_ascii_stl(const std::vector<std::uint8_t>& bytes) {
    std::string text(bytes.begin(), bytes.end());
    std::istringstream stream(text);
    std::vector<std::string> tokens;
    std::string token;
    while (stream >> token) {
        tokens.push_back(token);
    }
    std::vector<Triangle> triangles;
    std::size_t i = 0;
    while (i < tokens.size()) {
        if (tokens[i] == "facet" && i + 4 < tokens.size() && tokens[i + 1] == "normal") {
            Triangle tri;
            tri.normal = {std::strtof(tokens[i + 2].c_str(), nullptr),
                          std::strtof(tokens[i + 3].c_str(), nullptr),
                          std::strtof(tokens[i + 4].c_str(), nullptr)};
            int vert_index = 0;
            std::size_t j = i + 5;
            while (j < tokens.size() && tokens[j] != "endfacet" && vert_index < 3) {
                if (tokens[j] == "vertex" && j + 3 < tokens.size()) {
                    tri.verts[static_cast<std::size_t>(vert_index)] = {
                        std::strtof(tokens[j + 1].c_str(), nullptr),
                        std::strtof(tokens[j + 2].c_str(), nullptr),
                        std::strtof(tokens[j + 3].c_str(), nullptr)};
                    ++vert_index;
                    j += 4;
                } else {
                    ++j;
                }
            }
            if (vert_index == 3) {
                triangles.push_back(tri);
            }
            i = j;
        } else {
            ++i;
        }
    }
    return triangles;
}

std::array<float, 3> face_normal(const std::array<std::array<float, 3>, 3>& verts) {
    const std::array<float, 3> u{verts[1][0] - verts[0][0], verts[1][1] - verts[0][1], verts[1][2] - verts[0][2]};
    const std::array<float, 3> v{verts[2][0] - verts[0][0], verts[2][1] - verts[0][1], verts[2][2] - verts[0][2]};
    std::array<float, 3> n{u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]};
    const float len = std::sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
    if (len > 1e-12f) {
        return {n[0] / len, n[1] / len, n[2] / len};
    }
    return {0.0f, 0.0f, 1.0f};
}

std::optional<MeshAsset> bake_triangles(const std::vector<Triangle>& triangles, const std::string& mesh_id) {
    if (triangles.empty()) {
        return std::nullopt;
    }
    std::vector<float> positions;
    std::vector<float> normals;
    std::vector<std::int32_t> indices;
    std::map<std::array<std::uint32_t, 6>, std::int32_t> lookup;
    std::array<float, 3> bounds_min{{std::numeric_limits<float>::infinity(),
                                     std::numeric_limits<float>::infinity(),
                                     std::numeric_limits<float>::infinity()}};
    std::array<float, 3> bounds_max{{-std::numeric_limits<float>::infinity(),
                                     -std::numeric_limits<float>::infinity(),
                                     -std::numeric_limits<float>::infinity()}};

    for (const auto& tri : triangles) {
        std::array<float, 3> normal = tri.normal;
        if (normal[0] == 0.0f && normal[1] == 0.0f && normal[2] == 0.0f) {
            normal = face_normal(tri.verts);
        }
        for (const auto& vertex : tri.verts) {
            const std::array<std::uint32_t, 6> key{
                f32_bits(vertex[0]), f32_bits(vertex[1]), f32_bits(vertex[2]),
                f32_bits(normal[0]), f32_bits(normal[1]), f32_bits(normal[2])};
            auto it = lookup.find(key);
            std::int32_t index;
            if (it == lookup.end()) {
                index = static_cast<std::int32_t>(positions.size() / 3);
                positions.insert(positions.end(), vertex.begin(), vertex.end());
                normals.insert(normals.end(), normal.begin(), normal.end());
                lookup.emplace(key, index);
            } else {
                index = it->second;
            }
            indices.push_back(index);
            for (int axis = 0; axis < 3; ++axis) {
                bounds_min[static_cast<std::size_t>(axis)] =
                    std::min(bounds_min[static_cast<std::size_t>(axis)], vertex[static_cast<std::size_t>(axis)]);
                bounds_max[static_cast<std::size_t>(axis)] =
                    std::max(bounds_max[static_cast<std::size_t>(axis)], vertex[static_cast<std::size_t>(axis)]);
            }
        }
    }

    std::vector<std::uint8_t> position_bytes;
    position_bytes.reserve(positions.size() * 4);
    for (const float value : positions) {
        push_f32_le(position_bytes, value);
    }
    std::vector<std::uint8_t> normal_bytes;
    normal_bytes.reserve(normals.size() * 4);
    for (const float value : normals) {
        push_f32_le(normal_bytes, value);
    }
    std::vector<std::uint8_t> index_bytes;
    index_bytes.reserve(indices.size() * 4);
    for (const std::int32_t value : indices) {
        push_i32_le(index_bytes, value);
    }

    MeshAsset asset;
    asset.mesh_id = mesh_id;
    asset.vertex_count = positions.size() / 3;
    asset.triangle_count = triangles.size();
    asset.positions_b64 = base64_encode(position_bytes);
    asset.normals_b64 = base64_encode(normal_bytes);
    asset.indices_b64 = base64_encode(index_bytes);
    asset.bounds_min = bounds_min;
    asset.bounds_max = bounds_max;
    return asset;
}

std::optional<std::filesystem::path> resolve_mesh_path(const std::string& filename, const std::string& urdf_path) {
    std::string trimmed = filename;
    const auto first = trimmed.find_first_not_of(" \t");
    const auto last = trimmed.find_last_not_of(" \t");
    if (first == std::string::npos) {
        return std::nullopt;
    }
    trimmed = trimmed.substr(first, last - first + 1);

    if (trimmed.rfind("file://", 0) == 0) {
        return std::filesystem::path(trimmed.substr(7));
    }
    if (trimmed.rfind("package://", 0) == 0) {
        // package:// resolution needs a mesh root / ament index (Python only).
        return std::nullopt;
    }
    std::filesystem::path candidate(trimmed);
    if (candidate.is_absolute()) {
        return candidate;
    }
    const std::filesystem::path urdf(urdf_path);
    if (urdf.has_parent_path()) {
        return urdf.parent_path() / candidate;
    }
    return candidate;
}

std::string to_lower(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return value;
}

}  // namespace

std::string base64_encode(const std::vector<std::uint8_t>& data) {
    std::string out;
    out.reserve((data.size() + 2) / 3 * 4);
    for (std::size_t i = 0; i < data.size(); i += 3) {
        const std::uint32_t b0 = data[i];
        const std::uint32_t b1 = (i + 1 < data.size()) ? data[i + 1] : 0;
        const std::uint32_t b2 = (i + 2 < data.size()) ? data[i + 2] : 0;
        const std::uint32_t n = (b0 << 16) | (b1 << 8) | b2;
        out.push_back(kBase64Alphabet[(n >> 18) & 63]);
        out.push_back(kBase64Alphabet[(n >> 12) & 63]);
        out.push_back((i + 1 < data.size()) ? kBase64Alphabet[(n >> 6) & 63] : '=');
        out.push_back((i + 2 < data.size()) ? kBase64Alphabet[n & 63] : '=');
    }
    return out;
}

std::optional<MeshAsset> bake_stl_file(const std::string& path, const std::string& mesh_id) {
    std::ifstream file(path, std::ios::binary);
    if (!file) {
        return std::nullopt;
    }
    std::vector<std::uint8_t> bytes((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
    const auto triangles = is_binary_stl(bytes) ? parse_binary_stl(bytes) : parse_ascii_stl(bytes);
    return bake_triangles(triangles, mesh_id);
}

std::vector<MeshAsset> bake_visual_meshes(const std::string& urdf, const std::string& urdf_path) {
    std::vector<MeshAsset> assets;
    const std::regex visual_pattern(R"(<visual\b[\s\S]*?</visual>)");
    const std::regex mesh_pattern(R"(<mesh\b[^>]*filename\s*=\s*["']([^"']+)["'])");

    std::size_t block_index = 0;
    for (auto it = std::sregex_iterator(urdf.begin(), urdf.end(), visual_pattern);
         it != std::sregex_iterator();
         ++it, ++block_index) {
        const std::string block = it->str();
        for (auto mit = std::sregex_iterator(block.begin(), block.end(), mesh_pattern);
             mit != std::sregex_iterator();
             ++mit) {
            const std::string filename = (*mit)[1].str();
            if (to_lower(filename).size() < 4 ||
                to_lower(filename).compare(to_lower(filename).size() - 4, 4, ".stl") != 0) {
                continue;
            }
            const auto resolved = resolve_mesh_path(filename, urdf_path);
            if (!resolved) {
                continue;
            }
            const std::string stem = std::filesystem::path(filename).stem().string();
            const std::string mesh_id = (stem.empty() ? std::string("mesh") : stem) + "_" + std::to_string(block_index);
            if (auto asset = bake_stl_file(resolved->string(), mesh_id)) {
                assets.push_back(std::move(*asset));
            }
        }
    }
    return assets;
}

}  // namespace description
}  // namespace horus
