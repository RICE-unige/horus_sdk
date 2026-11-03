#include "horus/remote_rendering/datasets.hpp"
#include <Eigen/Geometry>
#include <algorithm>
#include <bit>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <map>
#include <openssl/sha.h>
#include <sstream>
#include <tinyxml2.h>
#include <unistd.h>

namespace horus::remote_rendering {
namespace {
namespace fs = std::filesystem;
const Json &catalog() {
    static const Json j = Json::parse(
#include "scene_catalog.inc"
    );
    return j;
}
fs::path cache(fs::path p) {
    if (!p.empty())
        return p;
    const auto home = getenv("HOME");
    if (!home)
        throw std::runtime_error("HOME is unset; supply a cache root");
    return fs::path(home) / ".cache/horus";
}
std::string line(std::istream &s, size_t maximum = 65536) {
    std::string out;
    char c;
    while (s.get(c)) {
        if (c == '\n')
            return out;
        if (out.size() == maximum)
            throw std::invalid_argument("Oversized PLY header line");
        out += c;
    }
    throw std::invalid_argument("Truncated PLY header");
}
struct Field {
    size_t offset;
    std::string type;
};
size_t scalar_size(const std::string &t) {
    if (t == "char" || t == "uchar" || t == "int8" || t == "uint8")
        return 1;
    if (t == "short" || t == "ushort" || t == "int16" || t == "uint16")
        return 2;
    if (t == "int" || t == "uint" || t == "int32" || t == "uint32" || t == "float" ||
        t == "float32")
        return 4;
    if (t == "double" || t == "float64")
        return 8;
    throw std::invalid_argument("Unsupported PLY scalar type: " + t);
}
template <class T> T number(const char *p, bool little) {
    std::array<char, sizeof(T)> b;
    std::copy_n(p, sizeof(T), b.begin());
    if (little != (std::endian::native == std::endian::little))
        std::reverse(b.begin(), b.end());
    T value;
    std::memcpy(&value, b.data(), sizeof(T));
    return value;
}
double scalar(const char *p, const Field &f, bool little) {
    p += f.offset;
    const auto &t = f.type;
    if (t == "float" || t == "float32")
        return number<float>(p, little);
    if (t == "double" || t == "float64")
        return number<double>(p, little);
    if (t == "char" || t == "int8")
        return number<int8_t>(p, little);
    if (t == "uchar" || t == "uint8")
        return number<uint8_t>(p, little);
    if (t == "short" || t == "int16")
        return number<int16_t>(p, little);
    if (t == "ushort" || t == "uint16")
        return number<uint16_t>(p, little);
    if (t == "int" || t == "int32")
        return number<int32_t>(p, little);
    return number<uint32_t>(p, little);
}
struct Reader {
    std::ifstream file;
    uint64_t remaining{};
    size_t stride{};
    bool little = true;
    std::map<std::string, Field> fields;
    std::vector<char> bytes;
    explicit Reader(const fs::path &path) : file(path, std::ios::binary) {
        if (!file)
            throw std::runtime_error("Cannot open point source: " + path.string());
        char magic[4]{};
        file.read(magic, 4);
        file.seekg(0);
        if (std::memcmp(magic, "HRL1", 4) == 0) {
            char h[20];
            file.read(h, 20);
            if (!file || number<uint32_t>(h + 4, true) != 1)
                throw std::invalid_argument("Invalid HRL header");
            const auto voxel = number<float>(h + 8, true);
            remaining = number<uint64_t>(h + 12, true);
            stride = 15;
            if (!std::isfinite(voxel) || voxel <= 0 || !remaining ||
                remaining > (UINT64_MAX - 20) / 15 || fs::file_size(path) != 20 + remaining * 15)
                throw std::invalid_argument("Invalid HRL size/voxel");
            fields = {{"x", {0, "float"}},    {"y", {4, "float"}},      {"z", {8, "float"}},
                      {"red", {12, "uchar"}}, {"green", {13, "uchar"}}, {"blue", {14, "uchar"}}};
            return;
        }
        auto first = line(file);
        if (first != "ply" && first != "ply\r")
            throw std::invalid_argument("Not a PLY/HRL point source");
        bool vertex = false, format = false, active = false;
        size_t total = first.size() + 1;
        while (true) {
            auto l = line(file);
            total += l.size() + 1;
            if (total > 65536)
                throw std::invalid_argument("PLY header exceeds 64 KiB");
            std::istringstream in(l);
            std::string k, t, n;
            in >> k;
            if (k == "format") {
                in >> t >> n;
                if (format || n != "1.0" ||
                    (t != "binary_little_endian" && t != "binary_big_endian"))
                    throw std::invalid_argument("Streaming requires binary PLY 1.0");
                format = true;
                little = t == "binary_little_endian";
            } else if (k == "element") {
                in >> n;
                uint64_t count;
                if (!(in >> count))
                    throw std::invalid_argument("Invalid PLY element");
                active = n == "vertex";
                if (active) {
                    if (vertex || !count || count > 10000000000ULL)
                        throw std::invalid_argument("Invalid PLY vertex count");
                    remaining = count;
                    vertex = true;
                } else if (!vertex && count)
                    throw std::invalid_argument("PLY vertices must precede other data");
            } else if (k == "property" && active) {
                in >> t >> n;
                if (n.empty() || fields.contains(n) || fields.size() >= 256)
                    throw std::invalid_argument("Invalid PLY property");
                fields[n] = {stride, t};
                stride += scalar_size(t);
            } else if (k == "end_header")
                break;
        }
        for (const auto *axis : {"x", "y", "z"})
            if (!fields.contains(axis))
                throw std::invalid_argument("PLY missing position");
        auto offset = uint64_t(file.tellg());
        if (!vertex || !format || !stride || remaining > (UINT64_MAX - offset) / stride ||
            fs::file_size(path) < offset + remaining * stride)
            throw std::invalid_argument("Truncated PLY vertex data");
    }
    SceneAsset next(size_t size) {
        auto n = std::min<uint64_t>({remaining, size, 64 * 1024 * 1024 / stride});
        bytes.resize(n * stride);
        file.read(bytes.data(), bytes.size());
        if (!file)
            throw std::runtime_error("Point source changed/truncated while streaming");
        remaining -= n;
        SceneAsset out;
        out.vertices.reserve(n);
        out.point_colors.reserve(n);
        const bool rgb =
            fields.contains("red") && fields.contains("green") && fields.contains("blue");
        for (size_t i = 0; i < n; ++i) {
            const auto *p = bytes.data() + i * stride;
            Point3f xyz{};
            for (size_t a = 0; a < 3; ++a)
                xyz[a] = scalar(p, fields.at(std::string(1, "xyz"[a])), little);
            if (!std::all_of(xyz.begin(), xyz.end(), [](float x) { return std::isfinite(x); }))
                continue;
            Rgb8 color{184, 188, 192};
            if (rgb) {
                size_t c = 0;
                for (const auto *name : {"red", "green", "blue"}) {
                    double v = scalar(p, fields.at(name), little);
                    if (!std::isfinite(v) || v < 0 || v > 255)
                        throw std::invalid_argument("Point RGB channel outside byte range");
                    color[c++] = uint8_t(v);
                }
            }
            out.vertices.push_back(xyz);
            out.point_colors.push_back(color);
        }
        return out;
    }
};
Eigen::Matrix4f matrix(const Json &j) {
    Eigen::Matrix4f m;
    for (size_t r = 0; r < 4; ++r)
        for (size_t c = 0; c < 4; ++c)
            m(r, c) = j.at(r).at(c).get<float>();
    if (!m.allFinite() || !m.row(3).isApprox(Eigen::RowVector4f(0, 0, 0, 1)))
        throw std::invalid_argument("Invalid scan affine matrix");
    return m;
}
fs::path scan_path(const fs::path &root, const std::string &name) {
    auto p = fs::canonical(root / name), r = fs::canonical(root);
    auto rel = p.lexically_relative(r);
    if (rel.empty() || *rel.begin() == ".." || !fs::is_regular_file(p))
        throw std::invalid_argument("Aligned scan escapes source root");
    return p;
}
Json identity() { return {{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}}; }
double quantile(std::vector<float> v, double q) {
    std::sort(v.begin(), v.end());
    double a = q * (v.size() - 1);
    size_t i = a;
    return v[i] + (double(v[std::min(i + 1, v.size() - 1)]) - v[i]) * (a - i);
}
template <class T> void write_le(std::ostream &out, T value) {
    std::array<char, sizeof(T)> b;
    std::memcpy(b.data(), &value, sizeof(T));
    if (std::endian::native != std::endian::little)
        std::reverse(b.begin(), b.end());
    out.write(b.data(), b.size());
}
} // namespace
Json scene_catalog(const fs::path &root) {
    auto j = catalog();
    for (auto &s : j["advanced"])
        s["cache_path"] = (cache(root) / s["cache_path"].get<std::string>()).string();
    return j;
}
fs::path resolve_scene_asset(const std::string &id, const fs::path &explicit_path,
                             const fs::path &root) {
    const auto j = scene_catalog(root);
    fs::path p = explicit_path;
    std::string kind;
    if (id == "cow_lady") {
        if (p.empty())
            p = cache(root) / "voxblox_cow_lady/extras/cow_and_lady_gt.ply";
        kind = "points";
    } else if (id == "courtyard") {
        if (p.empty())
            p = cache(root) / "eth3d_courtyard/scan_clean/courtyard/scan_clean";
        kind = "eth3d";
    } else if (j["eth3d"].contains(id)) {
        if (p.empty())
            p = cache(root) / "eth3d_remote_maps" / id / "scan_clean";
        kind = "eth3d";
    } else {
        for (const auto &s : j["advanced"])
            if (s["scene_id"] == id) {
                kind = s["kind"];
                if (p.empty())
                    p = s["cache_path"].get<std::string>();
                break;
            }
        if (kind.empty())
            throw std::invalid_argument("Unknown scene: " + id);
    }
    if (kind == "eth3d") {
        if (id == "courtyard" && fs::is_regular_file(p / "scan1.ply") &&
            fs::is_regular_file(p / "scan2.ply"))
            return fs::canonical(p);
        if (fs::is_regular_file(p / "scan_alignment.mlp"))
            return fs::canonical(p);
    } else if (fs::is_regular_file(p))
        return fs::canonical(p);
    if (fs::is_directory(p)) {
        std::vector<fs::path> candidates;
        size_t visited = 0;
        for (const auto &entry : fs::recursive_directory_iterator(p)) {
            if (++visited > 100000)
                throw std::length_error("Scene search exceeds 100000 entries");
            if (!entry.is_regular_file())
                continue;
            auto ext = entry.path().extension();
            if ((kind == "eth3d" && entry.path().filename() == "scan_alignment.mlp") ||
                (kind == "gaussian_splat" && ext == ".ply") ||
                (kind == "textured_triangle_mesh" &&
                 (ext == ".gltf" || ext == ".glb" || ext == ".obj" || ext == ".ply")))
                candidates.push_back(entry.path());
        }
        std::sort(candidates.begin(), candidates.end());
        if (!candidates.empty())
            return fs::canonical(kind == "eth3d" ? candidates.front().parent_path()
                                                 : candidates.front());
    }
    throw std::runtime_error("Scene asset is missing: " + p.string() +
                             "; provide downloaded source data");
}
Json prepare_eth3d_scene(const std::string &id, const fs::path &path, double scale,
                         const fs::path &cache_root) {
    if (!std::isfinite(scale) || scale < .1 || scale > 10 ||
        (!catalog()["eth3d"].contains(id) && id != "courtyard"))
        throw std::invalid_argument("Invalid ETH3D scene/scale");
    const auto root = resolve_scene_asset(id, path, cache_root);
    Json scans = Json::array();
    if (id == "courtyard")
        scans = catalog()["courtyard_scans"];
    else {
        if (fs::file_size(root / "scan_alignment.mlp") > 16 * 1024 * 1024)
            throw std::length_error("ETH3D alignment exceeds 16 MiB");
        tinyxml2::XMLDocument doc;
        if (doc.LoadFile((root / "scan_alignment.mlp").c_str()) != tinyxml2::XML_SUCCESS)
            throw std::invalid_argument("Invalid ETH3D alignment XML");
        auto *project = doc.FirstChildElement("MeshLabProject");
        auto *group = project ? project->FirstChildElement("MeshGroup") : nullptr;
        for (auto *e = group ? group->FirstChildElement("MLMesh") : nullptr; e;
             e = e->NextSiblingElement("MLMesh")) {
            auto *name = e->Attribute("filename");
            auto *m = e->FirstChildElement("MLMatrix44");
            if (!name || !m || !m->GetText())
                throw std::invalid_argument("Incomplete ETH3D alignment");
            std::istringstream in(m->GetText());
            Json a = Json::array();
            for (int r = 0; r < 4; ++r) {
                Json row = Json::array();
                for (int c = 0; c < 4; ++c) {
                    float v;
                    if (!(in >> v) || !std::isfinite(v))
                        throw std::invalid_argument("Invalid ETH3D matrix");
                    row.push_back(v);
                }
                a.push_back(row);
            }
            std::string extra;
            if (in >> extra)
                throw std::invalid_argument("Extra ETH3D matrix values");
            scans.push_back({{"filename", name}, {"alignment", a}});
        }
    }
    if (scans.empty() || scans.size() > 10000)
        throw std::invalid_argument("No scans or excessive ETH3D scans");
    std::array<double, 3> low{INFINITY, INFINITY, INFINITY}, high{-INFINITY, -INFINITY, -INFINITY};
    uint64_t count = 0;
    Json fingerprint = Json::array();
    for (const auto &s : scans) {
        auto source = scan_path(root, s["filename"]);
        auto transform = matrix(s["alignment"]);
        Reader reader(source);
        fingerprint.push_back({{"filename", s["filename"]},
                               {"bytes", fs::file_size(source)},
                               {"mtime", fs::last_write_time(source).time_since_epoch().count()},
                               {"alignment", s["alignment"]}});
        while (reader.remaining) {
            auto chunk = reader.next(100000);
            for (const auto &p : chunk.vertices) {
                const Eigen::Vector4f v = transform * Eigen::Vector4f(p[0], p[1], p[2], 1);
                if (!v.allFinite())
                    continue;
                ++count;
                for (size_t k = 0; k < 3; ++k) {
                    low[k] = std::min(low[k], double(v[k]));
                    high[k] = std::max(high[k], double(v[k]));
                }
            }
        }
    }
    if (!count)
        throw std::invalid_argument("No finite aligned points");
    const auto bytes = fingerprint.dump();
    unsigned char digest[SHA256_DIGEST_LENGTH];
    SHA256(reinterpret_cast<const unsigned char *>(bytes.data()), bytes.size(), digest);
    std::ostringstream hash;
    for (auto c : digest)
        hash << std::hex << std::setfill('0') << std::setw(2) << unsigned(c);
    std::array<float, 3> center{};
    std::array<double, 3> ext{};
    for (size_t k = 0; k < 3; ++k) {
        center[k] = (low[k] + high[k]) * .5;
        ext[k] = high[k] - low[k];
    }
    center[2] = low[2];
    double target =
        id == "courtyard" ? 32. : catalog()["eth3d"][id]["target_extent_m"].get<double>();
    target *= scale;
    const auto world = id == "courtyard"
                           ? .7 * scale
                           : target / std::max(1e-6, *std::max_element(ext.begin(), ext.end()));
    if (id == "courtyard")
        center = {10.45f, -1.84f, .14f};
    auto height = std::max(1.6, ext[2] * world * .55),
         distance = std::max(4., std::max(ext[0], ext[1]) * world * .35);
    auto diagonal = std::sqrt(ext[0] * ext[0] + ext[1] * ext[1] + ext[2] * ext[2]) * world;
    return {{"scene_id", id},
            {"root", root.string()},
            {"scans", scans},
            {"point_count", count},
            {"center", center},
            {"source_bounds_min", low},
            {"source_bounds_max", high},
            {"world_scale", world},
            {"depth_near_m", id == "courtyard" ? scale : std::max(.1, target / 250.)},
            {"depth_far_m", id == "courtyard" ? 60 * scale : std::max(40., diagonal * 2.5)},
            {"initial_position", {0., height, -distance}},
            {"initial_pitch_degrees",
             std::atan2(height - ext[2] * world * .3, distance) * 180 / std::acos(-1.)},
            {"fingerprint", hash.str()}};
}
SceneAsset prepare_cow_lady_scene(const fs::path &path, double scale) {
    if (!std::isfinite(scale) || scale <= 0)
        throw std::invalid_argument("Invalid Cow and Lady scale");
    SceneAsset out;
    PointChunkStream stream(path);
    while (auto chunk = stream.next()) {
        if (out.vertices.size() + chunk->vertices.size() > 100000000)
            throw std::length_error("Cow and Lady import exceeds 100 million points");
        out.vertices.insert(out.vertices.end(), chunk->vertices.begin(), chunk->vertices.end());
        out.point_colors.insert(out.point_colors.end(), chunk->point_colors.begin(),
                                chunk->point_colors.end());
    }
    if (out.vertices.empty())
        throw std::invalid_argument("Expected finite colored points");
    std::array<double, 3> center{};
    for (size_t k = 0; k < 3; ++k) {
        std::vector<float> axis;
        axis.reserve(out.vertices.size());
        for (const auto &p : out.vertices)
            axis.push_back(p[k]);
        center[k] = k == 2 ? quantile(axis, .01) : (quantile(axis, .01) + quantile(axis, .99)) * .5;
    }
    for (auto &p : out.vertices) {
        p = {float((p[0] - center[0]) * scale), float((p[2] - center[2]) * scale),
             float((p[1] - center[1]) * scale)};
        for (auto v : p)
            if (!std::isfinite(v))
                throw std::invalid_argument("Scaled point exceeds float32");
    }
    return out;
}
struct PointChunkStream::Impl {
    Json scene;
    size_t size, index{};
    std::unique_ptr<Reader> reader;
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    std::array<float, 3> center{};
    double scale = 1;
    bool convert = false;
    Impl(Json s, size_t n) : scene(std::move(s)), size(n) {
        if (!n || n > 1000000)
            throw std::invalid_argument("Chunk size must be 1..1000000");
        if (!scene.contains("file")) {
            convert = true;
            scale = scene.at("world_scale");
            center = scene.at("center").get<std::array<float, 3>>();
            if (!std::isfinite(scale) || scale <= 0)
                throw std::invalid_argument("Invalid prepared scale");
            for (auto x : center)
                if (!std::isfinite(x))
                    throw std::invalid_argument("Invalid prepared center");
            if (!scene.at("scans").is_array() || scene["scans"].empty() ||
                scene["scans"].size() > 10000)
                throw std::invalid_argument("Invalid scan list");
        }
    }
};
PointChunkStream::PointChunkStream(const Json &s, size_t n) : impl_(std::make_unique<Impl>(s, n)) {}
PointChunkStream::PointChunkStream(const fs::path &p, size_t n)
    : PointChunkStream(Json{{"file", p.string()}}, n) {}
PointChunkStream::~PointChunkStream() = default;
std::optional<SceneAsset> PointChunkStream::next() {
    auto &s = *impl_;
    while (true) {
        if (!s.reader || !s.reader->remaining) {
            if ((!s.convert && s.index) || (s.convert && s.index == s.scene["scans"].size()))
                return {};
            fs::path path;
            if (s.convert) {
                auto &scan = s.scene["scans"][s.index];
                path = scan_path(s.scene["root"].get<std::string>(), scan["filename"]);
                s.transform = matrix(scan["alignment"]);
            } else
                path = s.scene["file"].get<std::string>();
            s.reader = std::make_unique<Reader>(path);
            ++s.index;
        }
        auto out = s.reader->next(s.size);
        if (s.convert) {
            size_t keep = 0;
            for (size_t i = 0; i < out.vertices.size(); ++i) {
                auto p = out.vertices[i];
                const Eigen::Vector4f v = s.transform * Eigen::Vector4f(p[0], p[1], p[2], 1);
                if (!v.allFinite())
                    continue;
                p = {float((v[0] - s.center[0]) * s.scale), float((v[2] - s.center[2]) * s.scale),
                     float((v[1] - s.center[1]) * s.scale)};
                for (auto x : p)
                    if (!std::isfinite(x))
                        throw std::invalid_argument("Transformed point exceeds float32");
                out.vertices[keep] = p;
                out.point_colors[keep++] = out.point_colors[i];
            }
            out.vertices.resize(keep);
            out.point_colors.resize(keep);
        }
        if (!out.vertices.empty())
            return out;
    }
}
uint64_t write_point_lod(PointChunkStream &source, const fs::path &output, float voxel,
                         size_t maximum) {
    if (!std::isfinite(voxel) || voxel < .001f || voxel > .25f || !maximum || maximum > 20000000)
        throw std::invalid_argument("Invalid LOD voxel/budget");
    if (fs::exists(output))
        throw std::invalid_argument("LOD output already exists; refusing to overwrite");
    std::map<std::array<int32_t, 3>, std::pair<Point3f, Rgb8>> points;
    const float inverse = 1.f / voxel;
    while (auto chunk = source.next())
        for (size_t i = 0; i < chunk->vertices.size(); ++i) {
            auto p = chunk->vertices[i];
            std::array<int32_t, 3> key{};
            for (size_t k = 0; k < 3; ++k) {
                const double x = std::floor(p[k] * inverse);
                if (x < INT32_MIN || x > INT32_MAX)
                    throw std::invalid_argument("LOD voxel index overflow");
                key[k] = int32_t(x);
            }
            if (points.contains(key))
                continue;
            if (points.size() == maximum)
                throw std::length_error("LOD exceeds configured voxel memory budget");
            points.emplace(key, std::pair{p, chunk->point_colors[i]});
        }
    if (points.empty())
        throw std::invalid_argument("No finite LOD points");
    std::string pattern = output.string() + ".part.XXXXXX";
    std::vector<char> name(pattern.begin(), pattern.end());
    name.push_back(0);
    int fd = mkstemp(name.data());
    if (fd < 0)
        throw std::runtime_error("Cannot create LOD temporary file");
    ::close(fd);
    try {
        std::ofstream file(name.data(), std::ios::binary);
        file.write("HRL1", 4);
        write_le(file, uint32_t(1));
        write_le(file, voxel);
        write_le(file, uint64_t(points.size()));
        for (auto &[k, p] : points) {
            for (auto v : p.first)
                write_le(file, v);
            file.write(reinterpret_cast<const char *>(p.second.data()), 3);
        }
        file.close();
        if (!file)
            throw std::runtime_error("LOD write failed");
        // Hard-link publication is atomic and refuses a destination created by another writer.
        fs::create_hard_link(name.data(), output);
        fs::remove(name.data());
    } catch (...) {
        fs::remove(name.data());
        throw;
    }
    return points.size();
}
} // namespace horus::remote_rendering
