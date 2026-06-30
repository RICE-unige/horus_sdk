#include "horus/experiments/metrics.hpp"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <sstream>
#include <stdexcept>

namespace horus {
namespace experiments {

namespace {

struct ClockAnchor {
    long long wall_ns;
    std::chrono::steady_clock::time_point mono;
};

const ClockAnchor& clock_anchor() {
    static const ClockAnchor anchor{
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::system_clock::now().time_since_epoch())
            .count(),
        std::chrono::steady_clock::now()};
    return anchor;
}

void append_escaped(std::string& out, const std::string& value) {
    out.push_back('"');
    for (const char c : value) {
        switch (c) {
            case '"': out += "\\\""; break;
            case '\\': out += "\\\\"; break;
            case '\n': out += "\\n"; break;
            case '\r': out += "\\r"; break;
            case '\t': out += "\\t"; break;
            default: out.push_back(c); break;
        }
    }
    out.push_back('"');
}

void skip_ws(const std::string& s, std::size_t& i) {
    while (i < s.size() && (s[i] == ' ' || s[i] == '\t' || s[i] == '\r' || s[i] == '\n')) {
        ++i;
    }
}

std::string parse_string(const std::string& s, std::size_t& i) {
    if (i >= s.size() || s[i] != '"') {
        throw std::runtime_error("expected string");
    }
    ++i;
    std::string out;
    while (i < s.size()) {
        const char c = s[i++];
        if (c == '"') {
            return out;
        }
        if (c == '\\' && i < s.size()) {
            const char esc = s[i++];
            switch (esc) {
                case '"': out.push_back('"'); break;
                case '\\': out.push_back('\\'); break;
                case '/': out.push_back('/'); break;
                case 'n': out.push_back('\n'); break;
                case 'r': out.push_back('\r'); break;
                case 't': out.push_back('\t'); break;
                case 'b': out.push_back('\b'); break;
                case 'f': out.push_back('\f'); break;
                case 'u': {
                    if (i + 4 <= s.size()) {
                        const int code = std::stoi(s.substr(i, 4), nullptr, 16);
                        i += 4;
                        if (code < 0x80) {
                            out.push_back(static_cast<char>(code));
                        }
                    }
                    break;
                }
                default: out.push_back(esc); break;
            }
        } else {
            out.push_back(c);
        }
    }
    throw std::runtime_error("unterminated string");
}

JsonField parse_scalar(const std::string& s, std::size_t& i) {
    if (s.compare(i, 4, "true") == 0) {
        i += 4;
        return JsonField{true};
    }
    if (s.compare(i, 5, "false") == 0) {
        i += 5;
        return JsonField{false};
    }
    if (s.compare(i, 4, "null") == 0) {
        i += 4;
        return JsonField{std::string()};
    }
    std::size_t start = i;
    bool is_float = false;
    while (i < s.size()) {
        const char c = s[i];
        if (c == ',' || c == '}' || c == ' ' || c == '\t' || c == '\r' || c == '\n') {
            break;
        }
        if (c == '.' || c == 'e' || c == 'E') {
            is_float = true;
        }
        ++i;
    }
    const std::string token = s.substr(start, i - start);
    if (token.empty()) {
        throw std::runtime_error("expected value");
    }
    if (is_float) {
        return JsonField{std::stod(token)};
    }
    return JsonField{static_cast<long long>(std::stoll(token))};
}

}  // namespace

long long now_ns() {
    const auto& anchor = clock_anchor();
    const auto delta = std::chrono::duration_cast<std::chrono::nanoseconds>(
                           std::chrono::steady_clock::now() - anchor.mono)
                           .count();
    return anchor.wall_ns + delta;
}

std::string serialize_ndjson_object(const JsonObject& object) {
    std::string out = "{";
    bool first = true;
    for (const auto& [key, value] : object) {  // std::map iterates sorted by key
        if (!first) {
            out.push_back(',');
        }
        first = false;
        append_escaped(out, key);
        out.push_back(':');
        std::visit(
            [&out](const auto& v) {
                using T = std::decay_t<decltype(v)>;
                if constexpr (std::is_same_v<T, std::string>) {
                    append_escaped(out, v);
                } else if constexpr (std::is_same_v<T, bool>) {
                    out += v ? "true" : "false";
                } else if constexpr (std::is_same_v<T, long long>) {
                    out += std::to_string(v);
                } else {
                    std::ostringstream oss;
                    oss << v;
                    out += oss.str();
                }
            },
            value);
    }
    out.push_back('}');
    return out;
}

JsonObject parse_ndjson_object(const std::string& line) {
    JsonObject object;
    std::size_t i = 0;
    skip_ws(line, i);
    if (i >= line.size() || line[i] != '{') {
        throw std::runtime_error("expected object");
    }
    ++i;
    skip_ws(line, i);
    if (i < line.size() && line[i] == '}') {
        return object;
    }
    while (i < line.size()) {
        skip_ws(line, i);
        const std::string key = parse_string(line, i);
        skip_ws(line, i);
        if (i >= line.size() || line[i] != ':') {
            throw std::runtime_error("expected ':'");
        }
        ++i;
        skip_ws(line, i);
        JsonField value = (i < line.size() && line[i] == '"')
                              ? JsonField{parse_string(line, i)}
                              : parse_scalar(line, i);
        object[key] = std::move(value);
        skip_ws(line, i);
        if (i < line.size() && line[i] == ',') {
            ++i;
            continue;
        }
        if (i < line.size() && line[i] == '}') {
            break;
        }
    }
    return object;
}

NdjsonEventWriter::NdjsonEventWriter(
    const std::string& path,
    std::string run_id,
    std::string experiment,
    std::string condition,
    std::string source)
    : run_id_(std::move(run_id)),
      experiment_(std::move(experiment)),
      condition_(std::move(condition)),
      source_(std::move(source)) {
    const std::filesystem::path file_path(path);
    if (file_path.has_parent_path()) {
        std::filesystem::create_directories(file_path.parent_path());
    }
    out_.open(path, std::ios::out | std::ios::trunc);
    if (!out_) {
        throw std::runtime_error("failed to open metrics file: " + path);
    }
}

void NdjsonEventWriter::write(JsonObject event, std::optional<long long> timestamp_ns) {
    event.emplace("timestamp_ns", static_cast<long long>(timestamp_ns.value_or(now_ns())));
    event.emplace("run_id", run_id_);
    event.emplace("experiment", experiment_);
    event.emplace("condition", condition_);
    event.emplace("source", source_);
    out_ << serialize_ndjson_object(event) << '\n';
}

void NdjsonEventWriter::flush() {
    out_.flush();
}

std::string field_string(const JsonObject& object, const std::string& key, const std::string& fallback) {
    const auto it = object.find(key);
    if (it == object.end()) {
        return fallback;
    }
    if (const auto* value = std::get_if<std::string>(&it->second)) {
        return *value;
    }
    return fallback;
}

double field_double(const JsonObject& object, const std::string& key, double fallback) {
    const auto it = object.find(key);
    if (it == object.end()) {
        return fallback;
    }
    if (const auto* d = std::get_if<double>(&it->second)) {
        return *d;
    }
    if (const auto* ll = std::get_if<long long>(&it->second)) {
        return static_cast<double>(*ll);
    }
    return fallback;
}

long long field_ll(const JsonObject& object, const std::string& key, long long fallback) {
    const auto it = object.find(key);
    if (it == object.end()) {
        return fallback;
    }
    if (const auto* ll = std::get_if<long long>(&it->second)) {
        return *ll;
    }
    if (const auto* d = std::get_if<double>(&it->second)) {
        return static_cast<long long>(*d);
    }
    return fallback;
}

}  // namespace experiments
}  // namespace horus
