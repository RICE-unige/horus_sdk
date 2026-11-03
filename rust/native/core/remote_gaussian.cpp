#include "horus/remote_rendering/gaussian.hpp"
#include "vendor/happly/happly.h"
#include <Eigen/Geometry>
#include <algorithm>
#include <charconv>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <set>
#include <sstream>

namespace horus::remote_rendering {
namespace {
// 3DGS is a scalar vertex table. Bound declared allocations before hapPLY reserves
// buffers, and reject lists/nonvertex data rather than silently interpreting a mesh.
void preflight(std::ifstream &file, const GaussianLoadOptions &o) {
    std::string line;
    std::uint64_t count = 0, properties = 0, bytes = 0, stride = 0;
    std::string encoding;
    bool vertex = false, format = false, end = false;
    std::set<std::string> names;
    std::array<char, 65538> buffer{};
    while (file.getline(buffer.data(), buffer.size())) {
        line = buffer.data();
        bytes += line.size() + 1;
        if (bytes > 64 * 1024)
            throw std::invalid_argument("Gaussian PLY header exceeds 64 KiB");
        std::istringstream tokens(line);
        std::string key, type, name, extra;
        tokens >> key;
        if (bytes == line.size() + 1 && key != "ply")
            throw std::invalid_argument("Not a PLY file");
        if (key == "format") {
            tokens >> type >> name;
            if (format || name != "1.0" ||
                (type != "ascii" && type != "binary_little_endian" && type != "binary_big_endian"))
                throw std::invalid_argument("Unsupported Gaussian PLY format");
            format = true;
            encoding = type;
        } else if (key == "element") {
            tokens >> name;
            if (vertex || name != "vertex" || !(tokens >> count) || !count ||
                count > o.maximum_gaussians)
                throw std::invalid_argument(
                    "Gaussian PLY must contain a bounded scalar vertex table");
            vertex = true;
        } else if (key == "property") {
            tokens >> type >> name;
            const std::set<std::string> types{
                "char", "uchar", "short", "ushort", "int",   "uint",   "float",   "double",
                "int8", "uint8", "int16", "uint16", "int32", "uint32", "float32", "float64"};
            if (!vertex || name.empty() || !types.contains(type) || !names.insert(name).second ||
                ++properties > 256 || (tokens >> extra))
                throw std::invalid_argument("Invalid Gaussian PLY scalar property");
            stride += (type == "double" || type == "float64") ? 8
                      : (type == "float" || type == "float32" || type == "int" || type == "uint" ||
                         type == "int32" || type == "uint32")
                          ? 4
                      : (type == "short" || type == "ushort" || type == "int16" || type == "uint16")
                          ? 2
                          : 1;
        } else if (key == "end_header") {
            end = true;
            break;
        }
    }
    // Parser storage, converted attributes and final contiguous arrays coexist.
    if (!end || !format || !vertex || !properties ||
        count > o.maximum_decoded_bytes / (properties * 24))
        throw std::invalid_argument(
            "Gaussian PLY exceeds decoded-memory budget or has an invalid header");
    if (encoding != "ascii") {
        const auto start = file.tellg();
        file.seekg(0, std::ios::end);
        const auto length = file.tellg();
        if (start < 0 || length < start || count > uint64_t(length - start) / stride)
            throw std::invalid_argument("Truncated Gaussian PLY vertex data");
    }
    file.clear();
    file.seekg(0);
}
double quantile(std::vector<float> values, double q) {
    std::sort(values.begin(), values.end());
    const auto x = q * (values.size() - 1);
    const auto lo = static_cast<std::size_t>(x), hi = std::min(lo + 1, values.size() - 1);
    return values[lo] + (values[hi] - double(values[lo])) * (x - lo);
}
Eigen::Quaterniond eigen(Quaternion q) { return {q.w, q.x, q.y, q.z}; }
} // namespace
EyeView GaussianScene::world_to_source(const EyeView &e) const {
    const auto q = eigen(source_from_world_rotation).normalized();
    const Eigen::Vector3d p =
        q * Eigen::Vector3d(e.position.x, e.position.y, e.position.z) +
        Eigen::Vector3d(source_from_world_translation.x, source_from_world_translation.y,
                        source_from_world_translation.z);
    const auto r = q * eigen(e.rotation);
    return {{p.x(), p.y(), p.z()}, {r.x(), r.y(), r.z(), r.w()}, e.fov};
}
GaussianScene load_gaussian_ply(const std::string &path, const GaussianLoadOptions &o) {
    if (!std::isfinite(o.world_scale) || o.world_scale <= 0 || !o.maximum_gaussians ||
        !o.maximum_decoded_bytes || !std::isfinite(o.floor_quantile) || o.floor_quantile < .9 ||
        o.floor_quantile > 1)
        throw std::invalid_argument("Invalid Gaussian loading scale, quantile or budget");
    std::ifstream file(path, std::ios::binary);
    if (!file)
        throw std::runtime_error("Cannot open Gaussian PLY: " + path);
    preflight(file, o);
    happly::PLYData data(file);
    auto &vertex = data.getElement("vertex");
    const auto property = [&](const std::string &name) { return vertex.getProperty<double>(name); };
    std::vector<std::string> rest;
    for (const auto &name : vertex.getPropertyNames())
        if (name.starts_with("f_rest_"))
            rest.push_back(name);
    for (std::size_t i = 0; i < rest.size(); ++i)
        if (!vertex.hasProperty("f_rest_" + std::to_string(i)))
            throw std::invalid_argument("Gaussian SH properties must be contiguous from f_rest_0");
    const auto coefficients = 1 + rest.size() / 3;
    const auto degree = static_cast<unsigned>(std::sqrt(coefficients)) - 1;
    if (rest.size() % 3 || (degree + 1) * (degree + 1) != coefficients || degree > 4)
        throw std::invalid_argument("Invalid or unsupported Gaussian SH degree (expected 0..4)");
    std::vector<std::vector<double>> columns;
    for (const auto *name : {"x", "y", "z", "scale_0", "scale_1", "scale_2", "rot_0", "rot_1",
                             "rot_2", "rot_3", "opacity", "f_dc_0", "f_dc_1", "f_dc_2"})
        columns.push_back(property(name));
    for (std::size_t i = 0; i < rest.size(); ++i)
        columns.push_back(property("f_rest_" + std::to_string(i)));
    GaussianScene s;
    s.sh_degree = degree;
    const auto count = columns.front().size();
    s.means.reserve(count);
    s.scales.reserve(count);
    s.quaternions.reserve(count);
    s.opacities.reserve(count);
    s.spherical_harmonics.reserve(count * coefficients);
    for (std::size_t i = 0; i < count; ++i) {
        Point3f mean{}, scale{};
        std::array<float, 4> quat{};
        bool finite = true;
        for (const auto &c : columns)
            finite &= std::isfinite(c.at(i));
        for (unsigned k = 0; k < 3; ++k) {
            mean[k] = columns[k][i] * o.world_scale;
            scale[k] = std::exp(std::clamp(columns[3 + k][i], -20., 10.)) * o.world_scale;
            finite &= std::isfinite(mean[k]) && std::isfinite(scale[k]) && scale[k] > 0;
        }
        double norm = 0;
        for (unsigned k = 0; k < 4; ++k)
            norm = std::hypot(norm, columns[6 + k][i]);
        for (unsigned k = 0; k < 4; ++k)
            quat[k] = norm < 1e-8 ? (k == 0 ? 1 : 0) : columns[6 + k][i] / norm;
        for (std::size_t k = 11; k < columns.size(); ++k)
            finite &= std::isfinite(static_cast<float>(columns[k][i]));
        if (!finite) {
            ++s.discarded_nonfinite;
            continue;
        }
        s.means.push_back(mean);
        s.scales.push_back(scale);
        s.quaternions.push_back(quat);
        s.opacities.push_back(1. / (1. + std::exp(-std::clamp(columns[10][i], -30., 30.))));
        s.spherical_harmonics.push_back(
            {float(columns[11][i]), float(columns[12][i]), float(columns[13][i])});
        for (std::size_t k = 0; k + 1 < coefficients; ++k)
            s.spherical_harmonics.push_back({float(columns[14 + k][i]),
                                             float(columns[14 + coefficients - 1 + k][i]),
                                             float(columns[14 + 2 * (coefficients - 1) + k][i])});
    }
    if (s.means.empty())
        throw std::invalid_argument("Gaussian PLY contains no finite Gaussians");
    std::array<std::vector<float>, 3> axes;
    for (const auto &p : s.means)
        for (unsigned k = 0; k < 3; ++k)
            axes[k].push_back(p[k]);
    if (o.canonicalize_y_down) {
        s.source_from_world_rotation = {0, 0, 1, 0};
        s.source_from_world_translation = {
            quantile(axes[0], .5), quantile(axes[1], o.floor_quantile), quantile(axes[2], .5)};
    }
    Eigen::Vector3d lo, hi;
    for (unsigned k = 0; k < 3; ++k) {
        lo[k] = quantile(axes[k], .005);
        hi[k] = quantile(axes[k], .995);
    }
    Eigen::Vector3d wlo = Eigen::Vector3d::Constant(INFINITY),
                    whi = Eigen::Vector3d::Constant(-INFINITY);
    const auto &t = s.source_from_world_translation;
    for (unsigned mask = 0; mask < 8; ++mask) {
        Eigen::Vector3d p;
        for (unsigned k = 0; k < 3; ++k)
            p[k] = (mask & (1u << k)) ? hi[k] : lo[k];
        p = eigen(s.source_from_world_rotation).conjugate() * (p - Eigen::Vector3d(t.x, t.y, t.z));
        wlo = wlo.cwiseMin(p);
        whi = whi.cwiseMax(p);
    }
    s.world_bounds_min = {wlo.x(), wlo.y(), wlo.z()};
    s.world_bounds_max = {whi.x(), whi.y(), whi.z()};
    return s;
}
} // namespace horus::remote_rendering
