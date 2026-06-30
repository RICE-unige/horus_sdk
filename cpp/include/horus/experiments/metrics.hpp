#ifndef HORUS_EXPERIMENTS_METRICS_HPP
#define HORUS_EXPERIMENTS_METRICS_HPP

#include <fstream>
#include <map>
#include <optional>
#include <string>
#include <variant>

namespace horus {
namespace experiments {

// A flat JSON value as used by HORUS experiment event logs.
using JsonField = std::variant<std::string, long long, double, bool>;
using JsonObject = std::map<std::string, JsonField>;

// Unix-epoch-like timestamp from a monotonic clock, anchored once to wall time
// (mirrors horus.experiments.metrics.now_ns): stays monotonic within the
// process even if wall time steps.
long long now_ns();

// Serialize a flat object as a single NDJSON line (keys sorted for stable
// cross-language diffs).
std::string serialize_ndjson_object(const JsonObject& object);

// Parse one NDJSON line produced by serialize_ndjson_object. Throws
// std::runtime_error on malformed input.
JsonObject parse_ndjson_object(const std::string& line);

// Write event records as newline-delimited JSON with stable common fields.
class NdjsonEventWriter {
public:
    NdjsonEventWriter(
        const std::string& path,
        std::string run_id,
        std::string experiment,
        std::string condition,
        std::string source);

    void write(JsonObject event, std::optional<long long> timestamp_ns = std::nullopt);
    void flush();

private:
    std::ofstream out_;
    std::string run_id_;
    std::string experiment_;
    std::string condition_;
    std::string source_;
};

// Helpers for reading flat event fields with defaults.
std::string field_string(const JsonObject& object, const std::string& key, const std::string& fallback = "");
double field_double(const JsonObject& object, const std::string& key, double fallback = 0.0);
long long field_ll(const JsonObject& object, const std::string& key, long long fallback = 0);

}  // namespace experiments
}  // namespace horus

#endif  // HORUS_EXPERIMENTS_METRICS_HPP
