#include "texture_color.hpp"
#include <algorithm>
#include <cstring>
#include <limits>
#include <memory>
#include <stdexcept>
#include <tiffio.h>
#include <webp/decode.h>
#define STB_IMAGE_IMPLEMENTATION
#define STBI_NO_STDIO
#define STBI_NO_PNG
#define STBI_NO_JPEG
#define STBI_NO_HDR
#define STBI_NO_LINEAR
#define STBI_MAX_DIMENSIONS 32768
#include "vendor/stb/stb_image.h"

namespace horus::description {
namespace {
constexpr std::size_t max_pixels = 64 * 1024 * 1024;
void dimensions(std::uint64_t w, std::uint64_t h) {
    if (!w || !h || w > 32768 || h > 32768 || w * h > max_pixels)
        throw std::length_error("Material texture exceeds pixel budget");
}
struct TiffInput {
    const unsigned char *data;
    std::size_t size, offset{};
};
tmsize_t read_tiff(thandle_t h, void *out, tmsize_t count) {
    auto &s = *static_cast<TiffInput *>(h);
    if (count < 0)
        return -1;
    auto n = std::min<std::size_t>(count, s.size - s.offset);
    std::memcpy(out, s.data + s.offset, n);
    s.offset += n;
    return n;
}
tmsize_t no_write(thandle_t, void *, tmsize_t) { return -1; }
toff_t seek_tiff(thandle_t h, toff_t offset, int whence) {
    auto &s = *static_cast<TiffInput *>(h);
    const auto base = whence == SEEK_SET   ? 0
                      : whence == SEEK_CUR ? s.offset
                      : whence == SEEK_END ? s.size
                                           : UINT64_MAX;
    if (base == UINT64_MAX)
        return toff_t(-1);
    // libtiff represents negative relative offsets using unsigned toff_t.
    const auto delta = static_cast<std::int64_t>(offset);
    if (delta < 0 && (delta == INT64_MIN || std::uint64_t(-delta) > base))
        return toff_t(-1);
    if (delta >= 0 && std::uint64_t(delta) > s.size - base)
        return toff_t(-1);
    s.offset = delta < 0 ? base - std::uint64_t(-delta) : base + std::uint64_t(delta);
    return s.offset;
}
int close_tiff(thandle_t) { return 0; }
toff_t size_tiff(thandle_t h) { return static_cast<TiffInput *>(h)->size; }
int no_map(thandle_t, void **, toff_t *) { return 0; }
void no_unmap(thandle_t, void *, toff_t) {}
TextureImage tiff(const unsigned char *data, std::size_t size) {
    TiffInput input{data, size};
    std::unique_ptr<TIFF, decltype(&TIFFClose)> decoder(
        TIFFClientOpen("material", "rm", &input, read_tiff, no_write, seek_tiff, close_tiff,
                       size_tiff, no_map, no_unmap),
        TIFFClose);
    if (!decoder)
        throw std::runtime_error("Cannot read TIFF material texture");
    std::uint32_t w = 0, h = 0;
    if (!TIFFGetField(decoder.get(), TIFFTAG_IMAGEWIDTH, &w) ||
        !TIFFGetField(decoder.get(), TIFFTAG_IMAGELENGTH, &h))
        throw std::runtime_error("TIFF material is missing dimensions");
    dimensions(w, h);
    std::vector<std::uint32_t> raster(std::size_t(w) * h);
    if (!TIFFReadRGBAImageOriented(decoder.get(), w, h, raster.data(), ORIENTATION_TOPLEFT, 1))
        throw std::runtime_error("Cannot decode TIFF material texture");
    TextureImage out{std::vector<unsigned char>(raster.size() * 4), 4, w, h};
    for (std::size_t i = 0; i < raster.size(); ++i) {
        const unsigned a = TIFFGetA(raster[i]);
        const unsigned channels[3] = {TIFFGetR(raster[i]), TIFFGetG(raster[i]),
                                      TIFFGetB(raster[i])};
        // TIFF's RGBA helper returns associated alpha. HORUS stores straight alpha.
        for (unsigned k = 0; k < 3; ++k)
            out.bytes[i * 4 + k] = a ? std::min(255u, (channels[k] * 255 + a / 2) / a) : 0;
        out.bytes[i * 4 + 3] = a;
    }
    return out;
}
} // namespace
TextureImage decode_other_texture(const unsigned char *data, std::size_t size) {
    if (size > INT_MAX || size < 4)
        throw std::invalid_argument("Invalid material texture size");
    int w = 0, h = 0, channels = 0;
    if (size >= 12 && std::memcmp(data, "RIFF", 4) == 0 && std::memcmp(data + 8, "WEBP", 4) == 0) {
        if (!WebPGetInfo(data, size, &w, &h))
            throw std::runtime_error("Invalid WebP texture header");
        dimensions(w, h);
        TextureImage out{std::vector<unsigned char>(std::size_t(w) * h * 4), 4, unsigned(w),
                         unsigned(h)};
        if (!WebPDecodeRGBAInto(data, size, out.bytes.data(), out.bytes.size(), w * 4))
            throw std::runtime_error("Cannot decode WebP material texture");
        return out;
    }
    if ((data[0] == 'I' && data[1] == 'I' && (data[2] == 42 || data[2] == 43) && data[3] == 0) ||
        (data[0] == 'M' && data[1] == 'M' && data[2] == 0 && (data[3] == 42 || data[3] == 43)))
        return tiff(data, size);
    if (!stbi_info_from_memory(data, size, &w, &h, &channels))
        throw std::invalid_argument("Unsupported/corrupt material texture "
                                    "(PNG/JPEG/BMP/TGA/GIF/PNM/PSD/WebP/TIFF supported)");
    dimensions(w, h);
    if (stbi_is_16_bit_from_memory(data, size))
        throw std::invalid_argument(
            "16-bit BMP/TGA/GIF/PNM/PSD textures require explicit 8-bit conversion");
    std::unique_ptr<stbi_uc, decltype(&stbi_image_free)> pixels(
        stbi_load_from_memory(data, size, &w, &h, &channels, 4), stbi_image_free);
    if (!pixels)
        throw std::runtime_error("Cannot decode material texture");
    return {std::vector<unsigned char>(pixels.get(), pixels.get() + std::size_t(w) * h * 4), 4,
            unsigned(w), unsigned(h)};
}
} // namespace horus::description
