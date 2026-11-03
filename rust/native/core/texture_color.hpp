#pragma once
#include <array>
#include <filesystem>
#include <vector>
struct aiScene;
struct aiString;
namespace horus::description {
struct TextureImage {
    std::vector<unsigned char> bytes;
    unsigned channels{}, width{}, height{};
};
TextureImage texture_image(const aiScene *scene, const aiString &texture,
                           const std::filesystem::path &mesh);
std::array<double, 3> texture_color(const aiScene *scene, const aiString &texture,
                                    const std::filesystem::path &mesh);
} // namespace horus::description
