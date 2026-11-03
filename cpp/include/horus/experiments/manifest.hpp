#pragma once
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <nlohmann/json.hpp>
#include <sstream>
#include <string>
#include <sys/utsname.h>

namespace horus::experiments {
using Json = nlohmann::json;
inline std::string utc_stamp(std::chrono::system_clock::time_point time, const char *format) {
    const auto stamp = std::chrono::system_clock::to_time_t(time);
    std::tm utc{};
    if (!gmtime_r(&stamp, &utc))
        throw std::invalid_argument("Invalid experiment timestamp");
    std::ostringstream out;
    out << std::put_time(&utc, format);
    return out.str();
}
inline std::string
default_run_id(const std::string &experiment, const std::string &condition, int repetition = 1,
               std::chrono::system_clock::time_point time = std::chrono::system_clock::now()) {
    auto slug = [](const std::string &value) {
        std::string out;
        for (unsigned char c : value) {
            if ((c >= 'a' && c <= 'z') || (c >= '0' && c <= '9'))
                out += c;
            else if (c >= 'A' && c <= 'Z')
                out += char(c + ('a' - 'A'));
            else if (!out.empty() && out.back() != '_')
                out += '_';
        }
        if (!out.empty() && out.back() == '_')
            out.pop_back();
        return out.empty() ? std::string("run") : out;
    };
    std::ostringstream out;
    out << slug(experiment) << '_' << slug(condition) << "_r" << std::setfill('0') << std::setw(2)
        << repetition << '_' << utc_stamp(time, "%Y%m%d_%H%M%S");
    return out.str();
}
struct RunIdentity {
    std::string run_id, experiment, condition;
    int repetition = 1;
};
struct ExperimentManifest {
    Json fields;
    static ExperimentManifest from_identity(const RunIdentity &id,
                                            const Json &options = Json::object()) {
        if (!options.is_object())
            throw std::invalid_argument("Manifest options must be an object");
        struct utsname system {};
        const bool known = uname(&system) == 0;
        Json j{{"run_id", id.run_id},
               {"experiment", id.experiment},
               {"condition", id.condition},
               {"repetition", id.repetition},
               {"date", utc_stamp(std::chrono::system_clock::now(), "%Y-%m-%dT%H:%M:%S+00:00")},
               {"duration_s", 120.0},
               {"warmup_s", 10.0},
               {"robot_count", 1},
               {"operator_count", 1},
               {"stream_count", 0},
               {"transport", "ros2"},
               {"topology", "lan"},
               {"resolution", ""},
               {"target_fps", 0.0},
               {"robot_profile", "minimal"},
               {"map_profile", "none"},
               {"notes", ""},
               {"source_git_commit", ""},
               {"host", known ? system.nodename : ""},
               {"platform",
                known ? std::string(system.sysname) + "-" + system.release + "-" + system.machine
                      : ""},
               {"extra", Json::object()}};
        for (const auto &[key, value] : options.items()) {
            if (!j.contains(key) || key == "run_id" || key == "experiment" || key == "condition" ||
                key == "repetition")
                throw std::invalid_argument("Unknown or identity manifest override: " + key);
            j[key] = value;
        }
        return {std::move(j)};
    }
    Json to_dict() const { return fields; }
    std::filesystem::path save(const std::filesystem::path &directory) const {
        std::filesystem::create_directories(directory);
        auto path = directory / "run_manifest.json";
        std::ofstream out;
        out.exceptions(std::ios::badbit | std::ios::failbit);
        out.open(path);
        out << fields.dump(2) << '\n';
        out.close();
        return path;
    }
};
} // namespace horus::experiments
