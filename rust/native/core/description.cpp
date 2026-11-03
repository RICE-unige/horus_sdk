#include "horus/description/compiler.hpp"
#include "horus/teaching/config.hpp"
#include "texture_color.hpp"
#include <Eigen/Geometry>
#include <assimp/Importer.hpp>
#include <assimp/config.h>
#include <assimp/material.h>
#include <assimp/postprocess.h>
#include <assimp/scene.h>
#include <bit>
#include <cerrno>
#include <chrono>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <fcntl.h>
#include <filesystem>
#include <fstream>
#include <functional>
#include <map>
#include <meshoptimizer.h>
#include <openssl/evp.h>
#include <optional>
#include <poll.h>
#include <set>
#include <spawn.h>
#include <sstream>
#include <stdexcept>
#include <sys/wait.h>
#include <tinyxml2.h>
#include <unistd.h>
#include <vector>
#include <zlib.h>

extern char **environ;

namespace horus::description {
namespace {
using Json = nlohmann::json;
using Element = tinyxml2::XMLElement;
using Vec = Eigen::Vector3d;
using Transform = Eigen::Affine3d;
namespace fs = std::filesystem;
constexpr std::size_t max_payload_bytes = 256 * 1024 * 1024;
std::string xacro(const fs::path &path) {
    struct Pipes {
        int out[2]{-1, -1}, err[2]{-1, -1};
        ~Pipes() {
            for (int fd : {out[0], out[1], err[0], err[1]})
                if (fd >= 0)
                    close(fd);
        }
    } pipes;
    if (pipe2(pipes.out, O_CLOEXEC) || pipe2(pipes.err, O_CLOEXEC))
        throw std::runtime_error("Cannot create xacro pipes");
    posix_spawn_file_actions_t actions;
    posix_spawn_file_actions_init(&actions);
    posix_spawnattr_t attrs;
    posix_spawnattr_init(&attrs);
    struct End {
        posix_spawn_file_actions_t *a;
        posix_spawnattr_t *s;
        ~End() {
            posix_spawn_file_actions_destroy(a);
            posix_spawnattr_destroy(s);
        }
    } end{&actions, &attrs};
    posix_spawn_file_actions_adddup2(&actions, pipes.out[1], STDOUT_FILENO);
    posix_spawn_file_actions_adddup2(&actions, pipes.err[1], STDERR_FILENO);
    posix_spawnattr_setflags(&attrs, POSIX_SPAWN_SETPGROUP);
    posix_spawnattr_setpgroup(&attrs, 0);
    std::string filename = fs::absolute(path).string();
    char program[] = "xacro";
    char *args[] = {program, filename.data(), nullptr};
    pid_t pid;
    const int error = posix_spawnp(&pid, program, &actions, &attrs, args, environ);
    if (error)
        throw std::runtime_error(
            "xacro is unavailable; source ROS with xacro installed, or supply a resolved URDF");
    struct Child {
        pid_t pid;
        bool reaped = false;
        ~Child() {
            if (!reaped) {
                kill(-pid, SIGKILL);
                while (waitpid(pid, nullptr, 0) < 0 && errno == EINTR) {
                }
            }
        }
    } child{pid};
    close(pipes.out[1]);
    pipes.out[1] = -1;
    close(pipes.err[1]);
    pipes.err[1] = -1;
    for (int fd : {pipes.out[0], pipes.err[0]})
        if (fcntl(fd, F_SETFL, fcntl(fd, F_GETFL) | O_NONBLOCK) < 0)
            throw std::runtime_error("Cannot read xacro output");
    std::string output, errors;
    bool open[2]{true, true};
    int status = 0;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(20);
    while (open[0] || open[1] || !child.reaped) {
        if (std::chrono::steady_clock::now() > deadline)
            throw std::runtime_error("xacro exceeded 20-second deadline: " + errors);
        pollfd fds[2]{{open[0] ? pipes.out[0] : -1, POLLIN, 0},
                      {open[1] ? pipes.err[0] : -1, POLLIN, 0}};
        if (poll(fds, 2, 20) < 0 && errno != EINTR)
            throw std::runtime_error("xacro pipe polling failed");
        for (int i = 0; i < 2; ++i)
            if (open[i]) {
                char buffer[8192];
                const auto n = read(fds[i].fd, buffer, sizeof(buffer));
                if (n > 0) {
                    auto &text = i == 0 ? output : errors;
                    text.append(buffer, n);
                    if (i == 0 && text.size() > 16 * 1024 * 1024)
                        throw std::length_error("Expanded URDF exceeds XML budget");
                    if (i == 1 && text.size() > 8192)
                        text.erase(0, text.size() - 8192);
                } else if (n == 0)
                    open[i] = false;
                else if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR)
                    throw std::runtime_error("Cannot read xacro output");
            }
        if (!child.reaped) {
            const auto waited = waitpid(pid, &status, WNOHANG);
            if (waited == pid)
                child.reaped = true;
            else if (waited < 0 && errno != EINTR)
                throw std::runtime_error("Cannot wait for xacro");
        }
    }
    if (!WIFEXITED(status) || WEXITSTATUS(status) != 0)
        throw std::runtime_error("xacro failed: " + errors);
    return output;
}
std::string attr(const Element *e, const char *key, const std::string &fallback = "") {
    const auto *value = e ? e->Attribute(key) : nullptr;
    return value ? value : fallback;
}
const Element *child(const Element *e, const char *tag) {
    return e ? e->FirstChildElement(tag) : nullptr;
}
Vec vector(const std::string &value, Vec fallback = Vec::Zero()) {
    if (value.empty())
        return fallback;
    Vec out;
    std::istringstream in(value);
    std::string extra;
    if (!(in >> out.x() >> out.y() >> out.z()) || (in >> extra) || !out.allFinite())
        throw std::invalid_argument("Invalid finite URDF vector: " + value);
    return out;
}
Json array(const Vec &v) { return Json::array({v.x(), v.y(), v.z()}); }
Transform origin(const Element *e) {
    auto t = Transform::Identity();
    const auto *o = child(e, "origin");
    const Vec angles = vector(attr(o, "rpy"));
    t.linear() =
        (Eigen::AngleAxisd(angles.z(), Vec::UnitZ()) * Eigen::AngleAxisd(angles.y(), Vec::UnitY()) *
         Eigen::AngleAxisd(angles.x(), Vec::UnitX()))
            .toRotationMatrix();
    t.translation() = vector(attr(o, "xyz"));
    return t;
}
Vec rpy(const Transform &t) {
    const auto &r = t.linear();
    const double pitch = std::atan2(-r(2, 0), std::hypot(r(0, 0), r(1, 0)));
    if (std::abs(std::cos(pitch)) < 1e-8)
        return {std::atan2(-r(1, 2), r(1, 1)), pitch, 0};
    return {std::atan2(r(2, 1), r(2, 2)), pitch, std::atan2(r(1, 0), r(0, 0))};
}
void pose(Json &value, const Transform &t) {
    value["origin_xyz"] = array(t.translation());
    value["origin_rpy"] = array(rpy(t));
}
double positive(const Element *e, const char *key) {
    const std::string raw = attr(e, key);
    std::size_t used = 0;
    double n = std::stod(raw, &used);
    if (used != raw.size() || !std::isfinite(n) || n <= 0)
        throw std::invalid_argument("Invalid URDF dimension");
    return n;
}
std::string base64(const std::string &data) {
    if (data.size() > max_payload_bytes)
        throw std::length_error("Robot model exceeds native byte budget");
    std::string encoded(4 * ((data.size() + 2) / 3) + 1, '\0');
    int written = EVP_EncodeBlock(reinterpret_cast<unsigned char *>(encoded.data()),
                                  reinterpret_cast<const unsigned char *>(data.data()),
                                  static_cast<int>(data.size()));
    if (written < 0)
        throw std::runtime_error("Model base64 encoding failed");
    encoded.resize(written);
    return encoded;
}
void word(std::string &out, std::uint32_t value) {
    for (unsigned i = 0; i < 4; ++i)
        out.push_back(static_cast<char>(value >> (8 * i)));
}
void number(std::string &out, double value) {
    float f = static_cast<float>(value);
    if (!std::isfinite(f))
        throw std::invalid_argument("Non-finite mesh coordinate");
    word(out, std::bit_cast<std::uint32_t>(f));
}
fs::path expand(const std::string &text) {
    std::string expanded;
    for (std::size_t i = 0; i < text.size();) {
        if (text[i] != '$') {
            expanded += text[i++];
            continue;
        }
        const auto start = i++;
        const bool braced = i < text.size() && text[i] == '{';
        if (braced) ++i;
        const auto name_start = i;
        while (i < text.size() && (std::isalnum(static_cast<unsigned char>(text[i])) || text[i] == '_')) ++i;
        const auto name = text.substr(name_start, i - name_start);
        if (braced && (i == text.size() || text[i] != '}')) {
            expanded += text.substr(start, i - start);
            continue;
        }
        if (braced) ++i;
        const char *value = name.empty() ? nullptr : std::getenv(name.c_str());
        expanded += value ? value : text.substr(start, i - start);
    }
    if (expanded.starts_with("~/")) {
        const char *home = std::getenv("HOME");
        if (home)
            return fs::path(home) / expanded.substr(2);
    }
    return expanded;
}
fs::path resolve(const std::string &uri, const Json &options) {
    std::vector<fs::path> paths;
    auto parent = expand(options.value("urdf_path", "")).parent_path();
    if (uri.starts_with("package://")) {
        const std::string relative = uri.substr(10);
        const auto slash = relative.find('/');
        if (slash == std::string::npos)
            throw std::invalid_argument("Invalid package mesh URI");
        const auto package = relative.substr(0, slash), suffix = relative.substr(slash + 1);
        for (auto p = parent; !p.empty() && p != p.parent_path(); p = p.parent_path()) {
            if (p.filename() == package)
                paths.push_back(p / suffix);
        }
        const auto *env = std::getenv("AMENT_PREFIX_PATH");
        std::istringstream prefixes(env ? env : "");
        std::string prefix;
        while (std::getline(prefixes, prefix, ':'))
            if (!prefix.empty())
                paths.push_back(fs::path(prefix) / "share" / relative);
        if (options.contains("mesh_root") && !options.at("mesh_root").get<std::string>().empty()) {
            const auto root = expand(options.at("mesh_root"));
            paths.push_back(root / relative);
            paths.push_back(root / suffix);
        }
    } else {
        const auto path = expand(uri.starts_with("file://") ? uri.substr(7) : uri);
        if (path.is_absolute()) paths.push_back(path);
        else {
            if (!parent.empty()) paths.push_back(parent / path);
            if (!options.value("mesh_root", "").empty())
                paths.push_back(expand(options.at("mesh_root")) / path);
        }
    }
    for (const auto &p : paths)
        if (fs::is_regular_file(p))
            return fs::canonical(p);
    throw std::runtime_error("Cannot resolve robot mesh: " + uri);
}
std::optional<Vec> rgba(const Element *material) {
    const auto *color = child(material, "color");
    if (!color)
        return {};
    std::istringstream in(attr(color, "rgba"));
    Vec v;
    double alpha;
    std::string extra;
    if (!(in >> v.x() >> v.y() >> v.z() >> alpha) || (in >> extra) || !v.allFinite() ||
        !std::isfinite(alpha))
        throw std::invalid_argument("Invalid URDF material color");
    return v.cwiseMax(0).cwiseMin(1);
}
std::optional<Vec> named_color(std::string name) {
    std::transform(name.begin(), name.end(), name.begin(),
                   [](unsigned char c) { return std::tolower(c); });
    if (name.starts_with("gazebo/"))
        name = name.substr(7);
    std::replace(name.begin(), name.end(), ' ', '_');
    if (name == "black")
        return Vec(0, 0, 0);
    if (name == "white")
        return Vec(1, 1, 1);
    if (name == "red")
        return Vec(0.8, 0, 0);
    if (name == "green")
        return Vec(0, 0.8, 0);
    if (name == "blue")
        return Vec(0, 0, 0.8);
    if (name == "yellow")
        return Vec(0.8, 0.8, 0);
    if (name == "orange")
        return Vec(1, 0.42, 0.04);
    if (name == "brown")
        return Vec(0.87, 0.81, 0.76);
    if (name == "silver")
        return Vec(0.91, 0.91, 0.85);
    if (name == "grey" || name == "gray")
        return Vec::Constant(0.4);
    if (name == "darkgrey" || name == "dark_grey" || name == "darkgray" || name == "dark_gray")
        return Vec::Constant(0.15);
    if (name == "lightgrey" || name == "light_grey" || name == "lightgray" || name == "light_gray")
        return Vec::Constant(0.65);
    return {};
}
struct Group {
    std::string positions, normals, indices, colors;
    Vec lo = Vec::Constant(INFINITY), hi = Vec::Constant(-INFINITY);
    std::size_t vertices = 0, triangles = 0;
};
std::uint32_t read_word(const std::string &s, std::size_t offset) {
    std::uint32_t value = 0;
    for (unsigned i = 0; i < 4; ++i)
        value |= std::uint32_t(static_cast<unsigned char>(s.at(offset + i))) << (8 * i);
    return value;
}
float optimize(Group &g, std::size_t target, float tolerance) {
    if (!g.triangles)
        return 0;
    struct Vertex {
        float position[3], normal[3];
        std::uint32_t color;
    };
    static_assert(sizeof(Vertex) == 28);
    std::vector<Vertex> vertices(g.vertices);
    std::vector<unsigned> indices(g.triangles * 3), remap(g.vertices);
    for (std::size_t i = 0; i < g.vertices; ++i) {
        for (unsigned axis = 0; axis < 3; ++axis) {
            vertices[i].position[axis] =
                std::bit_cast<float>(read_word(g.positions, i * 12 + axis * 4));
            vertices[i].normal[axis] =
                std::bit_cast<float>(read_word(g.normals, i * 12 + axis * 4));
        }
        vertices[i].color = read_word(g.colors, i * 4);
    }
    for (std::size_t i = 0; i < indices.size(); ++i)
        indices[i] = read_word(g.indices, i * 4);
    auto count = meshopt_generateVertexRemap(remap.data(), indices.data(), indices.size(),
                                             vertices.data(), vertices.size(), sizeof(Vertex));
    meshopt_remapIndexBuffer(indices.data(), indices.data(), indices.size(), remap.data());
    meshopt_remapVertexBuffer(vertices.data(), vertices.data(), vertices.size(), sizeof(Vertex),
                              remap.data());
    vertices.resize(count);
    float error = 0;
    if (target < g.triangles) {
        std::vector<std::array<float, 6>> attributes(count);
        for (std::size_t i = 0; i < count; ++i)
            for (unsigned k = 0; k < 3; ++k) {
                attributes[i][k] = vertices[i].normal[k];
                attributes[i][k + 3] = float((vertices[i].color >> (8 * k)) & 255) / 255;
            }
        std::vector<unsigned> simplified(indices.size());
        const float weights[6]{0.5f, 0.5f, 0.5f, 1, 1, 1};
        const auto size = meshopt_simplifyWithAttributes(
            simplified.data(), indices.data(), indices.size(), vertices[0].position, count,
            sizeof(Vertex), attributes[0].data(), sizeof(attributes[0]), weights, 6, nullptr,
            std::max<std::size_t>(1, target) * 3, tolerance, 0, &error);
        if (size >= 3) {
            simplified.resize(size);
            indices = std::move(simplified);
        }
    }
    meshopt_optimizeVertexCache(indices.data(), indices.data(), indices.size(), vertices.size());
    count = meshopt_optimizeVertexFetch(vertices.data(), indices.data(), indices.size(),
                                        vertices.data(), vertices.size(), sizeof(Vertex));
    vertices.resize(count);
    g.positions.clear();
    g.normals.clear();
    g.colors.clear();
    g.indices.clear();
    g.lo = Vec::Constant(INFINITY);
    g.hi = Vec::Constant(-INFINITY);
    for (const auto &v : vertices) {
        for (unsigned k = 0; k < 3; ++k) {
            number(g.positions, v.position[k]);
            number(g.normals, v.normal[k]);
        }
        word(g.colors, v.color);
        Vec p(v.position[0], v.position[1], v.position[2]);
        g.lo = g.lo.cwiseMin(p);
        g.hi = g.hi.cwiseMax(p);
    }
    for (auto index : indices)
        word(g.indices, index);
    g.vertices = count;
    g.triangles = indices.size() / 3;
    return error;
}
Transform matrix(const aiMatrix4x4 &a) {
    Eigen::Matrix4d m;
    m << a.a1, a.a2, a.a3, a.a4, a.b1, a.b2, a.b3, a.b4, a.c1, a.c2, a.c3, a.c4, a.d1, a.d2, a.d3,
        a.d4;
    return Transform(m);
}
void append(Group &out, const aiScene *scene, const aiNode *node, const Transform &parent,
            const std::optional<Vec> &override_color, const std::optional<Vec> &fallback,
            const std::vector<std::optional<Vec>> &material_colors, std::size_t max_triangles) {
    const Transform t = parent * matrix(node->mTransformation);
    if (!t.matrix().allFinite() || std::abs(t.linear().determinant()) < 1e-12)
        throw std::invalid_argument("Singular/non-finite mesh transform");
    const Eigen::Matrix3d normal_transform = t.linear().inverse().transpose();
    for (unsigned mi = 0; mi < node->mNumMeshes; ++mi) {
        const auto *m = scene->mMeshes[node->mMeshes[mi]];
        if (out.triangles + m->mNumFaces > max_triangles ||
            out.vertices + m->mNumVertices > INT32_MAX)
            throw std::length_error("Robot mesh exceeds configured triangle/vertex budget");
        Vec color = fallback.value_or(Vec(0.78, 0.78, 0.76));
        if (material_colors.at(m->mMaterialIndex))
            color = *material_colors.at(m->mMaterialIndex);
        if (override_color)
            color = *override_color;
        const auto offset = static_cast<std::uint32_t>(out.vertices);
        for (unsigned v = 0; v < m->mNumVertices; ++v) {
            const auto &p = m->mVertices[v];
            const Vec position = t * Vec(p.x, p.y, p.z);
            Vec normal(0, 0, 1);
            if (m->HasNormals()) {
                const auto &n = m->mNormals[v];
                normal = normal_transform * Vec(n.x, n.y, n.z);
            }
            if (!normal.allFinite() || normal.norm() < 1e-12)
                throw std::invalid_argument("Invalid mesh normal");
            normal.normalize();
            for (int axis = 0; axis < 3; ++axis) {
                number(out.positions, position[axis]);
                number(out.normals, normal[axis]);
            }
            out.lo = out.lo.cwiseMin(position);
            out.hi = out.hi.cwiseMax(position);
            Vec c = color;
            if (!override_color && m->HasVertexColors(0)) {
                const auto &source = m->mColors[0][v];
                c = Vec(source.r, source.g, source.b);
            }
            if (!c.allFinite())
                throw std::invalid_argument("Non-finite mesh color");
            c = c.cwiseMax(0).cwiseMin(1);
            for (int axis = 0; axis < 3; ++axis)
                out.colors.push_back(static_cast<char>(std::nearbyint(c[axis] * 255)));
            out.colors.push_back(static_cast<char>(255));
        }
        const bool reversed = t.linear().determinant() < 0;
        for (unsigned fi = 0; fi < m->mNumFaces; ++fi) {
            const auto &f = m->mFaces[fi];
            if (f.mNumIndices != 3)
                throw std::invalid_argument("Robot visual includes non-triangle primitives");
            for (unsigned n : {0U, reversed ? 2U : 1U, reversed ? 1U : 2U}) {
                if (f.mIndices[n] >= m->mNumVertices)
                    throw std::invalid_argument("Invalid mesh index");
                word(out.indices, offset + f.mIndices[n]);
            }
        }
        out.vertices += m->mNumVertices;
        out.triangles += m->mNumFaces;
        if (out.positions.size() + out.normals.size() + out.indices.size() + out.colors.size() >
            max_payload_bytes / 2)
            throw std::length_error("Robot group exceeds byte budget");
    }
    for (unsigned i = 0; i < node->mNumChildren; ++i)
        append(out, scene, node->mChildren[i], t, override_color, fallback, material_colors,
               max_triangles);
}
} // namespace

std::string encode_description(const std::string &payload) {
    if (payload.size() > max_payload_bytes)
        throw std::length_error("Robot description exceeds byte budget");
    z_stream stream{};
    if (deflateInit2(&stream, Z_DEFAULT_COMPRESSION, Z_DEFLATED, 15 + 16, 8, Z_DEFAULT_STRATEGY) !=
        Z_OK)
        throw std::runtime_error("gzip initialization failed");
    struct End {
        z_stream *s;
        ~End() { deflateEnd(s); }
    } end{&stream};
    std::string output(deflateBound(&stream, payload.size()), '\0');
    stream.next_in = reinterpret_cast<Bytef *>(const_cast<char *>(payload.data()));
    stream.avail_in = payload.size();
    stream.next_out = reinterpret_cast<Bytef *>(output.data());
    stream.avail_out = output.size();
    if (deflate(&stream, Z_FINISH) != Z_STREAM_END)
        throw std::runtime_error("gzip failed");
    output.resize(stream.total_out);
    return base64(output);
}
Json Artifact::to_dict() const {
    return {{"manifest", manifest},
            {"payload", payload},
            {"payload_json", payload_json},
            {"encoded_payload", encoded_payload}};
}

Artifact compile_urdf(const std::string &xml, const Json &options) {
    if (!options.is_object() || xml.size() > 16 * 1024 * 1024)
        throw std::invalid_argument("Invalid robot description options/XML size");
    tinyxml2::XMLDocument doc;
    if (doc.Parse(xml.data(), xml.size()) != tinyxml2::XML_SUCCESS)
        throw std::invalid_argument(std::string("Malformed URDF: ") + doc.ErrorStr());
    const auto *root = doc.FirstChildElement("robot");
    if (!root)
        throw std::invalid_argument("URDF has no robot element");
    std::map<std::string, const Element *> links;
    std::vector<std::string> order;
    for (auto *e = root->FirstChildElement("link"); e; e = e->NextSiblingElement("link")) {
        const auto name = attr(e, "name");
        if (name.empty() || !links.emplace(name, e).second)
            throw std::invalid_argument("Empty/duplicate URDF link");
        order.push_back(name);
    }
    if (links.empty())
        throw std::invalid_argument("URDF has no links");
    struct Joint {
        std::string name, type, parent, child;
        Transform local;
        Vec axis;
    };
    std::vector<Joint> joints;
    std::map<std::string, std::size_t> by_child;
    std::set<std::string> joint_names;
    for (auto *e = root->FirstChildElement("joint"); e; e = e->NextSiblingElement("joint")) {
        Joint j{attr(e, "name"),
                attr(e, "type", "fixed"),
                attr(child(e, "parent"), "link"),
                attr(child(e, "child"), "link"),
                origin(e),
                vector(attr(child(e, "axis"), "xyz"), Vec(0, 0, 1))};
        if (j.name.empty() || !joint_names.insert(j.name).second || !links.count(j.parent) ||
            !links.count(j.child) || j.parent == j.child || by_child.count(j.child))
            throw std::invalid_argument("Invalid/duplicate URDF joint tree");
        if (j.axis.norm() < 1e-8)
            j.axis = Vec(0, 0, 1);
        else
            j.axis.normalize();
        by_child[j.child] = joints.size();
        joints.push_back(std::move(j));
    }
    std::string tree_root;
    for (const auto &name : order)
        if (!by_child.count(name)) {
            if (!tree_root.empty())
                throw std::invalid_argument("Disconnected URDF roots");
            tree_root = name;
        }
    if (tree_root.empty())
        throw std::invalid_argument("Cyclic URDF tree");
    std::map<std::string, std::vector<std::size_t>> children;
    for (std::size_t i = 0; i < joints.size(); ++i)
        children[joints[i].parent].push_back(i);
    std::map<std::string, Transform> absolute{{tree_root, Transform::Identity()}};
    std::vector<std::string> traversal{tree_root};
    for (std::size_t i = 0; i < traversal.size(); ++i)
        for (auto index : children[traversal[i]]) {
            const auto &j = joints[index];
            if (absolute.count(j.child))
                throw std::invalid_argument("Cyclic URDF");
            absolute.emplace(j.child, absolute.at(j.parent) * j.local);
            traversal.push_back(j.child);
        }
    if (absolute.size() != links.size())
        throw std::invalid_argument("Disconnected/cyclic URDF tree");
    std::string base = options.value("base_frame", tree_root);
    if (!links.count(base))
        base = tree_root;
    const Transform inverse = absolute.at(base).inverse();
    for (auto &[name, t] : absolute)
        t = inverse * t;
    std::string mode = options.value("body_mesh_mode", "preview_mesh");
    if (mode == "max_quality_mesh")
        mode = "runtime_high_mesh";
    if (mode != "preview_mesh" && mode != "runtime_high_mesh" && mode != "collision_only")
        throw std::invalid_argument("Unknown body_mesh_mode");
    const std::string pose_source = options.value("visual_link_pose_source", "static");
    if (pose_source != "static" && pose_source != "tf")
        throw std::invalid_argument("Unknown visual_link_pose_source");
    const bool visuals = options.value("include_visual_meshes", true) && mode != "collision_only";
    // Source admission is separate from the requested visualization budget.
    const auto budget = options.value("max_source_triangles", 5000000LL);
    if (budget < 1 || budget > 20000000)
        throw std::invalid_argument("max_source_triangles must be in [1, 20000000]");
    std::map<std::string, Vec> materials;
    for (auto *e = root->FirstChildElement("material"); e; e = e->NextSiblingElement("material")) {
        auto c = rgba(e);
        if (!c)
            c = named_color(attr(e, "name"));
        if (c)
            materials.insert_or_assign(attr(e, "name"), *c);
    }
    std::map<std::string, Vec> fallbacks;
    for (auto *e = root->FirstChildElement("gazebo"); e; e = e->NextSiblingElement("gazebo")) {
        const auto *material = child(e, "material");
        if (!material || !material->GetText())
            continue;
        const std::string name = material->GetText();
        auto color =
            materials.count(name) ? std::optional<Vec>(materials.at(name)) : named_color(name);
        if (color)
            fallbacks.insert_or_assign(attr(e, "reference"), *color);
    }
    Json payload = {{"version", "v2"},
                    {"robot_name", options.value("robot_name", attr(root, "name", "robot"))},
                    {"base_frame", base},
                    {"visual_link_pose_source", pose_source},
                    {"links", Json::array()},
                    {"joints", Json::array()},
                    {"visual_links", Json::array()},
                    {"mesh_assets", Json::array()}};
    std::map<std::string, Group> groups;
    std::map<std::string, std::shared_ptr<Assimp::Importer>> imports;
    std::map<std::string, std::vector<std::optional<Vec>>> source_colors;
    std::size_t collisions = 0, admitted_triangles = 0;
    for (const auto &name : order) {
        const auto *link = links.at(name);
        Json compiled = {{"name", name}, {"collisions", Json::array()}};
        for (auto *e = link->FirstChildElement("collision"); e;
             e = e->NextSiblingElement("collision")) {
            const auto *g = child(e, "geometry");
            Json value;
            pose(value, absolute.at(name) * origin(e));
            if (const auto *shape = child(g, "box")) {
                Vec size = vector(attr(shape, "size"));
                if ((size.array() <= 0).any())
                    throw std::invalid_argument("Invalid collision box");
                value["type"] = "box";
                value["size_xyz"] = array(size);
            } else if (const auto *shape = child(g, "sphere")) {
                value["type"] = "sphere";
                value["radius"] = positive(shape, "radius");
            } else if (const auto *shape = child(g, "cylinder")) {
                value["type"] = "cylinder";
                value["radius"] = positive(shape, "radius");
                value["length"] = positive(shape, "length");
            } else if (const auto *shape = child(g, "capsule")) {
                value["type"] = "capsule";
                value["radius"] = positive(shape, "radius");
                value["length"] = positive(shape, "length");
            } else if (const auto *shape = child(g, "mesh")) {
                const auto size =
                    options.value("mesh_proxy_size", std::array<double, 3>{0.28, 0.20, 0.18});
                Vec proxy = Vec(size[0], size[1], size[2])
                                .cwiseProduct(vector(attr(shape, "scale"), Vec::Ones()))
                                .cwiseAbs()
                                .cwiseMax(0.03);
                value["type"] = "mesh_proxy";
                value["mesh_uri"] = attr(shape, "filename");
                value["proxy_size_xyz"] = array(proxy);
            } else
                throw std::invalid_argument("Unsupported URDF collision geometry");
            compiled["collisions"].push_back(std::move(value));
            ++collisions;
        }
        payload["links"].push_back(std::move(compiled));
        if (!visuals)
            continue;
        auto group = name;
        while (by_child.count(group)) {
            const auto &j = joints[by_child.at(group)];
            if (j.type != "fixed")
                break;
            group = j.parent;
        }
        for (auto *e = link->FirstChildElement("visual"); e; e = e->NextSiblingElement("visual")) {
            const auto *mesh = child(child(e, "geometry"), "mesh");
            if (!mesh)
                continue;
            const auto path = resolve(attr(mesh, "filename"), options).string();
            auto &importer = imports[path];
            if (!importer) {
                importer = std::make_shared<Assimp::Importer>();
                importer->SetPropertyBool(AI_CONFIG_IMPORT_COLLADA_IGNORE_UP_DIRECTION, true);
                importer->SetPropertyBool(AI_CONFIG_IMPORT_COLLADA_IGNORE_UNIT_SIZE, true);
                // CAD exports can include helper edges/points alongside the visual surface.
                importer->SetPropertyInteger(AI_CONFIG_PP_SBP_REMOVE,
                                             aiPrimitiveType_POINT | aiPrimitiveType_LINE);
                const auto *scene = importer->ReadFile(
                    path, aiProcess_Triangulate | aiProcess_SortByPType |
                              aiProcess_GenSmoothNormals | aiProcess_JoinIdenticalVertices |
                              aiProcess_ImproveCacheLocality | aiProcess_ValidateDataStructure);
                if (!scene)
                    throw std::runtime_error("Cannot import " + path + ": " +
                                             importer->GetErrorString());
                for (unsigned i = 0; i < scene->mNumMaterials; ++i) {
                    std::optional<Vec> color;
                    aiColor4D diffuse;
                    const auto *material = scene->mMaterials[i];
                    aiString texture, material_name;
                    material->Get(AI_MATKEY_NAME, material_name);
                    if (material->Get(AI_MATKEY_COLOR_DIFFUSE, diffuse) == AI_SUCCESS &&
                        std::string(material_name.C_Str()) != "DefaultMaterial")
                        color = Vec(diffuse.r, diffuse.g, diffuse.b);
                    if (material->GetTexture(aiTextureType_DIFFUSE, 0, &texture) == AI_SUCCESS ||
                        material->GetTexture(aiTextureType_BASE_COLOR, 0, &texture) == AI_SUCCESS) {
                        const auto sampled = texture_color(scene, texture, path);
                        color = Vec(sampled[0], sampled[1], sampled[2]);
                    }
                    source_colors[path].push_back(color);
                }
            }
            const auto *scene = importer->GetScene();
            if (!scene || !scene->mRootNode)
                throw std::runtime_error("Empty robot mesh");
            auto color = rgba(child(e, "material"));
            const auto material_name = attr(child(e, "material"), "name");
            if (!color && materials.count(material_name))
                color = materials.at(material_name);
            if (!color)
                color = named_color(material_name);
            Transform transform = absolute.at(group).inverse() * absolute.at(name) * origin(e);
            transform.scale(vector(attr(mesh, "scale"), Vec::Ones()));
            auto &geometry = groups[group];
            const auto previous = geometry.triangles;
            append(geometry, scene, scene->mRootNode, transform, color,
                   fallbacks.count(name) ? std::optional<Vec>(fallbacks.at(name)) : std::nullopt,
                   source_colors.at(path), budget - admitted_triangles + previous);
            admitted_triangles += geometry.triangles - previous;
        }
    }
    for (const auto &j : joints) {
        Json value = {{"name", j.name},
                      {"type", j.type},
                      {"parent_link", j.parent},
                      {"child_link", j.child},
                      {"axis_xyz", array(j.axis)}};
        pose(value, absolute.at(j.child));
        payload["joints"].push_back(std::move(value));
    }
    std::size_t encoded_bytes = 0, total_triangles = 0, source_triangles = 0;
    for (const auto &[name, g] : groups)
        source_triangles += g.triangles;
    if (source_triangles > static_cast<std::size_t>(budget))
        throw std::length_error("Robot exceeds configured total source triangle budget");
    const auto requested = std::clamp(
        options.value("visual_mesh_triangle_budget", mode == "runtime_high_mesh" ? 240000 : 90000),
        2000, 500000);
    float maximum_error = 0;
    std::set<std::string> asset_ids;
    for (auto &[name, g] : groups) {
        if (g.triangles == 0)
            continue;
        const auto target =
            options.value("preserve_source_mesh", false)
                ? g.triangles
                : std::max<std::size_t>(mode == "runtime_high_mesh" ? 5000 : 500,
                                        static_cast<std::size_t>(
                                            std::ceil(double(requested) * g.triangles /
                                                      std::max<std::size_t>(1, source_triangles))));
        maximum_error = std::max(maximum_error,
                                 optimize(g, target, mode == "runtime_high_mesh" ? 0.001f : 0.01f));
        total_triangles += g.triangles;
        const auto id =
            "sha256:" + horus::teaching::sha256(g.positions + g.normals + g.indices + g.colors);
        Json asset = {{"mesh_id", id},
                      {"vertex_count", g.vertices},
                      {"triangle_count", g.triangles},
                      {"positions_b64", base64(g.positions)},
                      {"normals_b64", base64(g.normals)},
                      {"indices_b64", base64(g.indices)},
                      {"colors_b64", base64(g.colors)},
                      {"bounds_min", array(g.lo)},
                      {"bounds_max", array(g.hi)}};
        if (asset_ids.insert(id).second) {
            for (const auto *key : {"positions_b64", "normals_b64", "indices_b64", "colors_b64"})
                encoded_bytes += asset[key].get_ref<const std::string &>().size();
            payload["mesh_assets"].push_back(std::move(asset));
        }
        if (encoded_bytes > max_payload_bytes / 2)
            throw std::length_error("Robot exceeds encoded asset byte budget");
        Json visual = {{"name", name}, {"frame_id", name}, {"mesh_id", id}};
        pose(visual, absolute.at(name));
        payload["visual_links"].push_back(std::move(visual));
    }
    Artifact out;
    out.payload = std::move(payload);
    out.payload_json = horus::teaching::canonical(out.payload);
    out.encoded_payload = encode_description(out.payload_json);
    out.manifest = {
        {"version", "v2"},
        {"description_id", "sha256:" + horus::teaching::sha256(out.payload_json)},
        {"source", options.value("source", "local")},
        {"base_frame", base},
        {"link_count", links.size()},
        {"joint_count", joints.size()},
        {"collision_count", collisions},
        {"supports_collision", collisions > 0},
        {"supports_joints", !joints.empty()},
        {"supports_visual_meshes", !out.payload["mesh_assets"].empty()},
        {"mesh_asset_count", out.payload["mesh_assets"].size()},
        {"mesh_asset_encoded_bytes", encoded_bytes},
        {"is_transparent", options.value("is_transparent", false)},
        {"encoding", "json+gzip+base64"},
        {"chunk_size_bytes", std::clamp(options.value("chunk_size_bytes", 12000), 1024, 64000)},
        {"body_mesh_mode", mode}};
    out.manifest["source_triangle_count"] = source_triangles;
    out.manifest["triangle_count"] = total_triangles;
    out.manifest["requested_triangle_budget"] = requested;
    out.manifest["triangle_budget_met"] = total_triangles <= static_cast<std::size_t>(requested);
    out.manifest["simplification_max_relative_error"] = maximum_error;
    return out;
}
Artifact compile_file(const Json &options) {
    const auto path = expand(options.at("urdf_path").get<std::string>());
    if (!fs::is_regular_file(path) || fs::file_size(path) > 16 * 1024 * 1024)
        throw std::invalid_argument("Missing or oversized URDF: " + path.string());
    std::string xml;
    if (path.extension() == ".xacro")
        xml = xacro(path);
    else {
        std::ifstream in(path, std::ios::binary);
        xml.assign(std::istreambuf_iterator<char>(in), {});
    }
    auto normalized = options;
    normalized["urdf_path"] = fs::canonical(path).string();
    return compile_urdf(xml, normalized);
}
} // namespace horus::description
