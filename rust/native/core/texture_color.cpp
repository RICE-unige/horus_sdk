#include "texture_color.hpp"
#include <assimp/scene.h>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <jpeglib.h>
#include <memory>
#include <png.h>
#include <setjmp.h>
#include <stdexcept>
#include <vector>

namespace horus::description {
TextureImage decode_other_texture(const unsigned char *, std::size_t);
namespace {
constexpr std::size_t max_pixels = 64 * 1024 * 1024;
using Image = TextureImage;
struct JpegError {
    jpeg_error_mgr base;
    jmp_buf jump;
};
void jpeg_error(j_common_ptr c) { longjmp(reinterpret_cast<JpegError *>(c->err)->jump, 1); }
// libjpeg's C error boundary owns only C allocations until decompression ends.
unsigned char *decode_jpeg(const unsigned char *data, std::size_t size, std::size_t *bytes,
                           unsigned *width, unsigned *height) {
    auto *decoder =
        static_cast<jpeg_decompress_struct *>(std::calloc(1, sizeof(jpeg_decompress_struct)));
    auto *error = static_cast<JpegError *>(std::calloc(1, sizeof(JpegError)));
    auto *pixels = static_cast<unsigned char **>(std::calloc(1, sizeof(unsigned char *)));
    if (!decoder || !error || !pixels) {
        std::free(decoder);
        std::free(error);
        std::free(pixels);
        return nullptr;
    }
    decoder->err = jpeg_std_error(&error->base);
    error->base.error_exit = jpeg_error;
    if (setjmp(error->jump)) {
        if (decoder->mem)
            jpeg_destroy_decompress(decoder);
        std::free(*pixels);
        std::free(pixels);
        std::free(error);
        std::free(decoder);
        return nullptr;
    }
    jpeg_create_decompress(decoder);
    jpeg_mem_src(decoder, data, size);
    jpeg_read_header(decoder, TRUE);
    if (!decoder->image_width || !decoder->image_height ||
        std::size_t(decoder->image_width) * decoder->image_height > max_pixels)
        jpeg_error(reinterpret_cast<j_common_ptr>(decoder));
    decoder->out_color_space = JCS_RGB;
    jpeg_start_decompress(decoder);
    *bytes = std::size_t(decoder->output_width) * decoder->output_height * 3;
    *width = decoder->output_width;
    *height = decoder->output_height;
    *pixels = static_cast<unsigned char *>(std::malloc(*bytes));
    if (!*pixels)
        jpeg_error(reinterpret_cast<j_common_ptr>(decoder));
    while (decoder->output_scanline < decoder->output_height) {
        unsigned char *row =
            *pixels + std::size_t(decoder->output_scanline) * decoder->output_width * 3;
        jpeg_read_scanlines(decoder, &row, 1);
    }
    jpeg_finish_decompress(decoder);
    jpeg_destroy_decompress(decoder);
    auto *result = *pixels;
    std::free(pixels);
    std::free(error);
    std::free(decoder);
    return result;
}
Image decode(const unsigned char *data, std::size_t size) {
    if (size >= 8 && png_sig_cmp(data, 0, 8) == 0) {
        png_image image{};
        image.version = PNG_IMAGE_VERSION;
        if (!png_image_begin_read_from_memory(&image, data, size))
            throw std::runtime_error("Cannot read PNG material texture");
        struct End {
            png_image *p;
            ~End() { png_image_free(p); }
        } end{&image};
        if (!image.width || !image.height || std::size_t(image.width) * image.height > max_pixels)
            throw std::length_error("Material texture exceeds pixel budget");
        image.format = PNG_FORMAT_RGBA;
        Image out{std::vector<unsigned char>(PNG_IMAGE_SIZE(image)), 4, image.width, image.height};
        if (!png_image_finish_read(&image, nullptr, out.bytes.data(), 0, nullptr))
            throw std::runtime_error("Cannot decode PNG material texture");
        return out;
    }
    if (size >= 2 && data[0] == 255 && data[1] == 216) {
        std::size_t count = 0;
        unsigned width = 0, height = 0;
        auto *pixels = decode_jpeg(data, size, &count, &width, &height);
        if (!pixels)
            throw std::runtime_error("Cannot decode JPEG material texture");
        std::unique_ptr<unsigned char, decltype(&std::free)> owned(pixels, std::free);
        return {std::vector<unsigned char>(pixels, pixels + count), 3, width, height};
    }
    return decode_other_texture(data, size);
}
std::array<double, 3> average(const Image &image) {
    double weight = 0;
    std::array<double, 3> sum{};
    for (std::size_t i = 0; i < image.bytes.size(); i += image.channels) {
        const double alpha = image.channels == 4 ? image.bytes[i + 3] / 255.0 : 1.0;
        weight += alpha;
        for (unsigned k = 0; k < 3; ++k) {
            const double c = image.bytes[i + k] / 255.0;
            sum[k] += alpha * (c <= 0.04045 ? c / 12.92 : std::pow((c + 0.055) / 1.055, 2.4));
        }
    }
    if (weight == 0)
        return {0.78, 0.78, 0.76};
    for (auto &v : sum)
        v /= weight;
    return sum;
}
} // namespace
TextureImage texture_image(const aiScene *scene, const aiString &texture,
                           const std::filesystem::path &mesh) {
    if (const auto *embedded = scene->GetEmbeddedTexture(texture.C_Str())) {
        if (!embedded->mHeight) {
            if (embedded->mWidth > 128 * 1024 * 1024)
                throw std::length_error("Embedded texture exceeds byte budget");
            return decode(reinterpret_cast<const unsigned char *>(embedded->pcData),
                          embedded->mWidth);
        }
        if (std::size_t(embedded->mWidth) * embedded->mHeight > max_pixels)
            throw std::length_error("Embedded texture exceeds pixel budget");
        Image image{{}, 4, embedded->mWidth, embedded->mHeight};
        image.bytes.reserve(std::size_t(embedded->mWidth) * embedded->mHeight * 4);
        for (std::size_t i = 0; i < std::size_t(embedded->mWidth) * embedded->mHeight; ++i) {
            const auto &c = embedded->pcData[i];
            image.bytes.insert(image.bytes.end(), {c.r, c.g, c.b, c.a});
        }
        return image;
    }
    std::string name = texture.C_Str();
    if (name.starts_with("file://"))
        name = name.substr(7);
    auto path = std::filesystem::path(name);
    if (path.is_relative())
        path = mesh.parent_path() / path;
    if (!std::filesystem::is_regular_file(path) ||
        std::filesystem::file_size(path) > 128 * 1024 * 1024)
        throw std::runtime_error("Missing/oversized material texture: " + path.string());
    std::ifstream file(path, std::ios::binary);
    std::vector<unsigned char> bytes((std::istreambuf_iterator<char>(file)), {});
    return decode(bytes.data(), bytes.size());
}
std::array<double, 3> texture_color(const aiScene *scene, const aiString &texture,
                                    const std::filesystem::path &mesh) {
    return average(texture_image(scene, texture, mesh));
}
} // namespace horus::description
