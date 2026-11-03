#include "horus/experiments/metrics.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <limits>
#include <nlohmann/json.hpp>
#include <stdexcept>

namespace horus::experiments {
namespace {
std::string csv_escape(const std::string &value) {
  if (value.find_first_of(",\"\r\n") == std::string::npos)
    return value;
  std::string result = "\"";
  for (char c : value) {
    if (c == '"')
      result += '"';
    result += c;
  }
  return result + '"';
}
void open_writer(std::ofstream &out, const std::string &path) {
  const auto parent = std::filesystem::path(path).parent_path();
  if (!parent.empty())
    std::filesystem::create_directories(parent);
  out.exceptions(std::ios::badbit | std::ios::failbit);
  out.open(path);
}
nlohmann::json scalar(const JsonField &field) {
  return std::visit(
      [](const auto &v) -> nlohmann::json {
        if constexpr (std::is_same_v<std::decay_t<decltype(v)>, double>)
          if (!std::isfinite(v))
            throw std::invalid_argument("Metric must be finite");
        return v;
      },
      field);
}
} // namespace
long long now_ns() {
  static const auto anchor = std::pair{std::chrono::system_clock::now(),
                                       std::chrono::steady_clock::now()};
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             anchor.first.time_since_epoch() +
             (std::chrono::steady_clock::now() - anchor.second))
      .count();
}
std::string serialize_ndjson_object(const JsonObject &object) {
  auto result = nlohmann::json::object();
  for (const auto &[key, value] : object)
    result[key] = scalar(value);
  return result.dump();
}
JsonObject parse_ndjson_object(const std::string &line) {
  const auto value = nlohmann::json::parse(line);
  if (!value.is_object())
    throw std::invalid_argument("Metrics record must be an object");
  JsonObject out;
  for (const auto &[key, v] : value.items()) {
    if (v.is_string())
      out[key] = v.get<std::string>();
    else if (v.is_boolean())
      out[key] = v.get<bool>();
    else if (v.is_number_unsigned()) {
      if (v.get<uint64_t>() > uint64_t(std::numeric_limits<long long>::max()))
        throw std::out_of_range("Metric integer exceeds int64");
      out[key] = v.get<long long>();
    } else if (v.is_number_integer())
      out[key] = v.get<long long>();
    else if (v.is_number_float())
      out[key] = v.get<double>();
    else if (v.is_null())
      out[key] = std::string{};
    else
      throw std::invalid_argument("Use scalar values in flat metrics records");
  }
  return out;
}
NdjsonEventWriter::NdjsonEventWriter(const std::string &path,
                                     std::string run_id, std::string experiment,
                                     std::string condition, std::string source)
    : run_id_(std::move(run_id)), experiment_(std::move(experiment)),
      condition_(std::move(condition)), source_(std::move(source)) {
  open_writer(out_, path);
}
void NdjsonEventWriter::write(JsonObject event,
                              std::optional<long long> timestamp_ns) {
  event.try_emplace("timestamp_ns", timestamp_ns.value_or(now_ns()));
  event.try_emplace("run_id", run_id_);
  event.try_emplace("experiment", experiment_);
  event.try_emplace("condition", condition_);
  event.try_emplace("source", source_);
  out_ << serialize_ndjson_object(event) << '\n';
}
void NdjsonEventWriter::flush() { out_.flush(); }
CsvMetricWriter::CsvMetricWriter(const std::string &path, std::string run_id,
                                 std::string experiment, std::string condition,
                                 const std::vector<std::string> &names)
    : fields_{"timestamp_ns", "run_id", "experiment", "condition"},
      run_id_(std::move(run_id)), experiment_(std::move(experiment)),
      condition_(std::move(condition)) {
  open_writer(out_, path);
  for (const auto &name : names)
    if (std::find(fields_.begin(), fields_.end(), name) == fields_.end())
      fields_.push_back(name);
  for (size_t i = 0; i < fields_.size(); ++i)
    out_ << (i ? "," : "") << csv_escape(fields_[i]);
  out_ << '\n';
}
void CsvMetricWriter::write(JsonObject row,
                            std::optional<long long> timestamp_ns) {
  row.try_emplace("timestamp_ns", timestamp_ns.value_or(now_ns()));
  row.try_emplace("run_id", run_id_);
  row.try_emplace("experiment", experiment_);
  row.try_emplace("condition", condition_);
  std::string line;
  for (size_t i = 0; i < fields_.size(); ++i) {
    if (i)
      line += ',';
    if (const auto it = row.find(fields_[i]); it != row.end()) {
      const auto v = scalar(it->second);
      line += csv_escape(v.is_string() ? v.get<std::string>() : v.dump());
    }
  }
  out_ << line << '\n';
}
void CsvMetricWriter::flush() { out_.flush(); }
std::string field_string(const JsonObject &object, const std::string &key,
                         const std::string &fallback) {
  const auto it = object.find(key);
  if (it != object.end())
    if (const auto *v = std::get_if<std::string>(&it->second))
      return *v;
  return fallback;
}
double field_double(const JsonObject &object, const std::string &key,
                    double fallback) {
  const auto it = object.find(key);
  if (it != object.end()) {
    if (const auto *v = std::get_if<double>(&it->second))
      return *v;
    if (const auto *v = std::get_if<long long>(&it->second))
      return double(*v);
  }
  return fallback;
}
long long field_ll(const JsonObject &object, const std::string &key,
                   long long fallback) {
  const auto it = object.find(key);
  if (it != object.end()) {
    if (const auto *v = std::get_if<long long>(&it->second))
      return *v;
    if (const auto *v = std::get_if<double>(&it->second))
      if (std::isfinite(*v) &&
          *v >= double(std::numeric_limits<long long>::min()) &&
          *v < -double(std::numeric_limits<long long>::min()))
        return static_cast<long long>(*v);
  }
  return fallback;
}
} // namespace horus::experiments
