#include "horus/remote_rendering/scene.hpp"
#include "texture_color.hpp"
#include <algorithm>
#include <assimp/Importer.hpp>
#include <assimp/material.h>
#include <assimp/postprocess.h>
#include <assimp/scene.h>
#include <cmath>
#include <map>
#include <numeric>
#include <openssl/sha.h>

namespace horus::remote_rendering {
namespace {
struct Texture {
    std::uint32_t width{}, height{};
    std::vector<Rgb8> pixels;
    std::array<std::uint32_t, 4> rect{};
};
std::uint8_t byte(double v) {
    if (!std::isfinite(v))
        throw std::invalid_argument("Nonfinite scene color");
    return static_cast<std::uint8_t>(std::clamp(v * 255.0, 0.0, 255.0));
}
std::string key(const Texture &t) {
    unsigned char digest[SHA256_DIGEST_LENGTH];
    SHA256(reinterpret_cast<const unsigned char *>(t.pixels.data()), t.pixels.size() * 3, digest);
    return std::to_string(t.width) + ":" + std::to_string(t.height) + ":" +
           std::string(reinterpret_cast<const char *>(digest), sizeof(digest));
}
std::uint32_t align16(std::uint64_t n) {
    if (n > UINT32_MAX - 15)
        throw std::length_error("Scene texture atlas dimensions overflow");
    return (n + 15) / 16 * 16;
}
void atlas(std::vector<Texture> &textures, SceneAsset &out, std::uint64_t budget) {
    std::vector<std::size_t> order(textures.size());
    std::iota(order.begin(), order.end(), 0);
    std::stable_sort(order.begin(), order.end(), [&](auto a, auto b) {
        const auto &x = textures[a], &y = textures[b];
        return std::pair(x.height, x.width) > std::pair(y.height, y.width);
    });
    std::uint64_t area = 0, widest = 0;
    for (const auto &t : textures) {
        const std::uint64_t w = t.width + 4, h = t.height + 4;
        if (w * h > budget || area > budget - w * h)
            throw std::length_error("Scene atlas exceeds pixel budget");
        area += w * h;
        widest = std::max(widest, w);
    }
    const auto target =
        align16(std::max(widest, static_cast<std::uint64_t>(std::ceil(std::sqrt(double(area))))));
    std::uint64_t x = 0, y = 0, shelf = 0, used = 0;
    for (auto i : order) {
        auto &t = textures[i];
        if (x && x + t.width + 4 > target) {
            y += shelf;
            x = 0;
            shelf = 0;
        }
        if (y + t.height + 4 > budget)
            throw std::length_error("Scene atlas exceeds pixel budget");
        t.rect = {static_cast<std::uint32_t>(x + 2), static_cast<std::uint32_t>(y + 2), t.width,
                  t.height};
        x += t.width + 4;
        shelf = std::max(shelf, std::uint64_t(t.height) + 4);
        used = std::max(used, x);
    }
    out.texture_width = align16(used);
    out.texture_height = align16(y + shelf);
    if (std::uint64_t(out.texture_width) * out.texture_height > budget)
        throw std::length_error("Aligned scene atlas exceeds pixel budget");
    out.texture_atlas.resize(std::size_t(out.texture_width) * out.texture_height);
    for (const auto &t : textures) {
        for (int row = -2; row < int(t.height) + 2; ++row)
            for (int col = -2; col < int(t.width) + 2; ++col) {
                const auto src = std::size_t(std::clamp(row, 0, int(t.height) - 1)) * t.width +
                                 std::clamp(col, 0, int(t.width) - 1);
                const auto dst =
                    std::size_t(int(t.rect[1]) + row) * out.texture_width + int(t.rect[0]) + col;
                out.texture_atlas[dst] = t.pixels[src];
            }
    }
}
} // namespace

MeshInput SceneAsset::mesh_input() const {
    if (faces.empty())
        throw std::invalid_argument("Point-only scene has no triangle mesh");
    return {vertices,      faces,         face_colors,   uvs,
            texture_rects, texture_atlas, texture_width, texture_height};
}
SceneAsset load_scene(const std::filesystem::path &path, const SceneLoadOptions &o) {
    if (!std::isfinite(o.world_scale) || o.world_scale <= 0 || !o.maximum_vertices ||
        o.maximum_vertices > UINT32_MAX || !o.maximum_triangles ||
        o.maximum_triangles > UINT32_MAX || o.maximum_texture_pixels < 256 ||
        o.maximum_texture_pixels > 1024ULL * 1024 * 1024)
        throw std::invalid_argument("Invalid scene scale or admission budget");
    if (!std::filesystem::is_regular_file(path))
        throw std::invalid_argument("Scene file does not exist: " + path.string());
    Assimp::Importer importer;
    const auto *scene =
        importer.ReadFile(path.string(), aiProcess_Triangulate | aiProcess_SortByPType);
    if (!scene || !scene->mRootNode)
        throw std::runtime_error(std::string("Cannot import scene: ") + importer.GetErrorString());
    SceneAsset out;
    std::vector<Texture> textures;
    std::map<std::string, std::size_t> texture_keys;
    struct Material {
        std::size_t texture;
        aiColor4D tint;
    };
    std::vector<Material> materials;
    std::uint64_t texture_pixels = 0;
    for (unsigned i = 0; i < scene->mNumMaterials; ++i) {
        const auto *m = scene->mMaterials[i];
        aiColor4D tint(1, 1, 1, 1);
        if (aiGetMaterialColor(m, AI_MATKEY_BASE_COLOR, &tint) != AI_SUCCESS)
            aiGetMaterialColor(m, AI_MATKEY_COLOR_DIFFUSE, &tint);
        for (auto c : {tint.r, tint.g, tint.b})
            if (!std::isfinite(c))
                throw std::invalid_argument("Nonfinite material tint");
        Texture texture{4, 4, std::vector<Rgb8>(16, {255, 255, 255})};
        aiString name;
        if (m->GetTexture(aiTextureType_BASE_COLOR, 0, &name) == AI_SUCCESS ||
            m->GetTexture(aiTextureType_DIFFUSE, 0, &name) == AI_SUCCESS) {
            const auto image = description::texture_image(scene, name, path);
            texture.width = image.width;
            texture.height = image.height;
            texture.pixels.resize(std::size_t(image.width) * image.height);
            for (std::size_t p = 0; p < texture.pixels.size(); ++p)
                std::copy_n(image.bytes.data() + p * image.channels, 3, texture.pixels[p].begin());
        }
        const auto hash = key(texture);
        auto found = texture_keys.find(hash);
        std::size_t index;
        if (found == texture_keys.end()) {
            if (texture.pixels.size() > o.maximum_texture_pixels - texture_pixels)
                throw std::length_error("Scene textures exceed pixel budget");
            texture_pixels += texture.pixels.size();
            index = textures.size();
            texture_keys.emplace(hash, index);
            textures.push_back(std::move(texture));
        } else
            index = found->second;
        materials.push_back({index, tint});
    }
    std::vector<std::size_t> face_textures;
    struct Node {
        const aiNode *node;
        aiMatrix4x4 parent;
        unsigned depth;
    };
    std::vector<Node> nodes{{scene->mRootNode, aiMatrix4x4{}, 0}};
    std::size_t node_count = 0;
    while (!nodes.empty()) {
        auto [node, parent, depth] = nodes.back();
        nodes.pop_back();
        if (depth > 1024 || ++node_count > 1000000)
            throw std::length_error("Scene graph exceeds traversal budget");
        const auto transform = parent * node->mTransformation;
        for (unsigned i = 0; i < node->mNumChildren; ++i)
            nodes.push_back({node->mChildren[i], transform, depth + 1});
        for (unsigned k = 0; k < node->mNumMeshes; ++k) {
            const auto *m = scene->mMeshes[node->mMeshes[k]];
            if (m->mMaterialIndex >= materials.size())
                throw std::invalid_argument("Invalid scene material index");
            const auto material = materials[m->mMaterialIndex];
            if (m->mNumVertices > o.maximum_vertices - out.vertices.size())
                throw std::length_error("Scene exceeds vertex budget");
            const auto offset = out.vertices.size();
            for (unsigned v = 0; v < m->mNumVertices; ++v) {
                const auto p = transform * m->mVertices[v];
                Point3f point{float(p.x * o.world_scale), float(p.y * o.world_scale),
                              float(p.z * o.world_scale)};
                for (auto value : point)
                    if (!std::isfinite(value))
                        throw std::invalid_argument("Nonfinite transformed vertex");
                out.vertices.push_back(point);
                aiColor4D color = m->HasVertexColors(0) ? m->mColors[0][v] : aiColor4D(1, 1, 1, 1);
                out.point_colors.push_back({byte(color.r * material.tint.r),
                                            byte(color.g * material.tint.g),
                                            byte(color.b * material.tint.b)});
                const auto uv =
                    m->HasTextureCoords(0) ? m->mTextureCoords[0][v] : aiVector3D(0, 1, 0);
                if (!std::isfinite(uv.x) || !std::isfinite(uv.y))
                    throw std::invalid_argument("Nonfinite texture coordinate");
                out.uvs.push_back({uv.x, 1.0f - uv.y});
            }
            for (unsigned f = 0; f < m->mNumFaces; ++f) {
                const auto &face = m->mFaces[f];
                if (face.mNumIndices == 1)
                    continue;
                if (face.mNumIndices != 3)
                    throw std::invalid_argument("Scene contains non-triangle line geometry");
                if (out.faces.size() >= o.maximum_triangles)
                    throw std::length_error("Scene exceeds triangle budget");
                std::array<std::uint32_t, 3> indices;
                for (unsigned j = 0; j < 3; ++j) {
                    if (face.mIndices[j] >= m->mNumVertices)
                        throw std::invalid_argument("Invalid triangle index");
                    indices[j] = static_cast<std::uint32_t>(offset + face.mIndices[j]);
                }
                if (transform.Determinant() < 0)
                    std::swap(indices[1], indices[2]);
                const auto &a = out.vertices[indices[0]], &b = out.vertices[indices[1]],
                           &c = out.vertices[indices[2]];
                const aiVector3D ab(b[0] - a[0], b[1] - a[1], b[2] - a[2]),
                    ac(c[0] - a[0], c[1] - a[1], c[2] - a[2]);
                auto normal = ab ^ ac;
                const auto length = normal.Length();
                if (length > 1e-8f)
                    normal /= length;
                else
                    normal = {0, 0, 0};
                aiVector3D light(-.34f, .86f, -.38f);
                light.Normalize();
                const double illumination =
                    o.bake_lighting ? .78 + .22 * std::abs(normal * light) : 1.0;
                Rgb8 tint{};
                for (unsigned j = 0; j < 3; ++j) {
                    const double factor =
                        (out.point_colors[indices[0]][j] + out.point_colors[indices[1]][j] +
                         out.point_colors[indices[2]][j]) /
                        (3.0 * 255.0);
                    tint[j] =
                        byte(o.bake_lighting
                                 ? std::pow(std::clamp(factor * illumination, 0.0, 1.0), 1.0 / 2.2)
                                 : factor);
                }
                out.faces.push_back(indices);
                out.face_colors.push_back(tint);
                face_textures.push_back(material.texture);
            }
        }
    }
    if (out.vertices.empty())
        throw std::invalid_argument("Scene contains no vertices");
    if (out.faces.empty()) {
        out.texture_atlas = {{255, 255, 255}};
        return out;
    }
    atlas(textures, out, o.maximum_texture_pixels);
    out.texture_rects.reserve(face_textures.size());
    for (auto index : face_textures)
        out.texture_rects.push_back(textures[index].rect);
    return out;
}
} // namespace horus::remote_rendering
