#include <cmath>
#include <filesystem>
#include <fstream>
#include <horus/remote_rendering/gaussian.hpp>
#include <iostream>
#include <source_location>
#include <unistd.h>
using namespace horus::remote_rendering;
void require(bool b, std::source_location where = std::source_location::current()) {
    if (!b)
        throw std::runtime_error("Gaussian test failed at line " + std::to_string(where.line()));
}
template <class F> void rejects(F f) {
    bool threw = false;
    try {
        f();
    } catch (const std::exception &) {
        threw = true;
    }
    require(threw);
}
int main() {
    auto path = std::filesystem::temp_directory_path() /
                ("horus-gaussian-test-" + std::to_string(getpid()) + ".ply");
    const std::string header = "ply\nformat ascii 1.0\nelement vertex 2\n";
    {
        std::ofstream f(path);
        f << header;
        for (auto name : {"x", "y", "z", "scale_0", "scale_1", "scale_2", "rot_0", "rot_1", "rot_2",
                          "rot_3", "opacity", "f_dc_0", "f_dc_1", "f_dc_2"})
            f << "property float " << name << "\n";
        for (int i = 0; i < 9; ++i)
            f << "property float f_rest_" << i << "\n";
        f << "end_header\n1 2 3 0 0 0 2 0 0 0 0 1 2 3 10 11 12 20 21 22 30 31 32\n"
          << "4 5 6 0 0 0 0 0 0 0 0 4 5 6 40 41 42 50 51 52 60 61 62\n";
    }
    auto s = load_gaussian_ply(path.string(), {.world_scale = 2});
    require(s.means.size() == 2 && s.means[1][2] == 12 && s.scales[0][0] == 2 && s.sh_degree == 1);
    require(s.quaternions[0][0] == 1 && s.quaternions[1][0] == 1 && s.opacities[0] == .5);
    require(s.spherical_harmonics[1] == Point3f{10, 20, 30});
    require(s.spherical_harmonics[7] == Point3f{42, 52, 62});
    auto canonical = load_gaussian_ply(path.string(), {.canonicalize_y_down = true});
    require(canonical.source_from_world_translation.x == 2.5);
    auto eye = canonical.world_to_source({});
    require(eye.position.x == 2.5 && eye.rotation.z == 1);
    rejects([&] { load_gaussian_ply(path.string(), {.maximum_gaussians = 1}); });
    rejects([&] { load_gaussian_ply(path.string(), {.maximum_decoded_bytes = 16}); });
    rejects([&] { load_gaussian_ply(path.string(), {.world_scale = 0}); });
    rejects([&] { GaussianRenderer renderer("relative.so", s); });
    {
        std::ofstream f(path);
        f << "ply\nformat ascii 1.0\nelement vertex 999999999999999999\nend_header\n";
    }
    rejects([&] { load_gaussian_ply(path.string()); });
    {
        std::ofstream f(path);
        f << "ply\n" << std::string(70000, 'x') << "\nend_header\n";
    }
    rejects([&] { load_gaussian_ply(path.string()); });
    {
        std::ofstream f(path);
        f << "ply\nformat binary_little_endian 1.0\nelement vertex 20\nproperty float "
             "x\nend_header\n";
    }
    rejects([&] { load_gaussian_ply(path.string()); });
    {
        std::ofstream f(path);
        f << "ply\nformat ascii 1.0\nelement vertex 2\nproperty list uchar float x\nend_header\n";
    }
    rejects([&] { load_gaussian_ply(path.string()); });
    std::filesystem::remove(path);
    std::cout << "Gaussian loader/limits/transforms passed\n";
}
