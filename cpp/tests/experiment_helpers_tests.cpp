#include "horus/dataviz/collision_risk.hpp"
#include "horus/experiments/metrics.hpp"
#include "horus/experiments/workloads.hpp"
#include <cassert>
#include <unistd.h>
int main() {
  using namespace horus::experiments;
  const JsonObject precise{{"value", 0.123456789012345},
                           {"control", std::string("a\x01\tb")}};
  assert(parse_ndjson_object(serialize_ndjson_object(precise)) == precise);
  assert(field_string(parse_ndjson_object("{\"text\":\"\\u00e9\"}"), "text") ==
         "\xc3\xa9");
  for (const auto *bad : {"{\"x\":", "{}junk", "{\"x\":1oops}", "{\"x\":[]}"}) {
    bool rejected = false;
    try {
      parse_ndjson_object(bad);
    } catch (const std::exception &) {
      rejected = true;
    }
    assert(rejected);
  }
  assert(field_ll({{"x", INFINITY}}, "x", 7) == 7);
  assert(default_run_id("E0 Baseline", "one / two", 2,
                        std::chrono::system_clock::from_time_t(0)) ==
         "e0_baseline_one_two_r02_19700101_000000");
  const auto workload =
      workload_from_dict({{"robot_count", 3},
                          {"camera", {{"streams", 2}, {"fps", 30}}},
                          {"pointcloud", {{"points", 10}, {"hz", 2}}},
                          {"map", {{"representation", "mesh"}, {"hz", 1}}},
                          {"seed", 42},
                          {"extra", {{"test", true}}}});
  assert(workload.stream_count() == 8 && workload.extra.at("seed") == 42 &&
         workload.extra.at("test") == true);
  assert(workload_from_dict(workload.to_dict()).to_dict() ==
         workload.to_dict());
  const auto root = std::filesystem::temp_directory_path() /
                    ("horus-experiment-test-" + std::to_string(getpid()));
  const auto manifest = ExperimentManifest::from_identity(
      {"run", "E0", "base", 2}, {{"notes", "test"}});
  auto path = manifest.save(root);
  std::ifstream in(path);
  assert(Json::parse(in) == manifest.to_dict());
  in.close();
  {
    CsvMetricWriter csv((root / "metric.csv").string(), "run,one", "E0", "a\nb",
                        {"fps", "note", "fps"});
    csv.write({{"fps", 30LL}, {"note", std::string("say \"hi\"\r")}}, 0);
    csv.flush();
  }
  std::ifstream csv(root / "metric.csv");
  std::string text((std::istreambuf_iterator<char>(csv)), {});
  csv.close();
  assert(text == "timestamp_ns,run_id,experiment,condition,fps,note\n0,\"run,"
                 "one\",E0,\"a\nb\","
                 "30,\"say \"\"hi\"\"\r\"\n");
  using namespace horus::dataviz;
  const std::array<double, 4> ranges{NAN, 2., .5, INFINITY};
  auto scan = analyze_laser_scan<double>(ranges, 0, .5, 0, 10);
  assert(scan.min_distance_m == .5 &&
         std::abs(scan.direction[0] - std::cos(1.)) < 1e-9);
  const std::array<std::array<double, 3>, 3> points{
      {{.1, 0, 2}, {.3, .4, 0}, {NAN, 0, 0}}};
  auto cloud = analyze_point_cloud<double>(points, .35);
  assert(cloud.min_distance_m == .5 &&
         std::abs(cloud.direction[0] - .6) < 1e-9);
  assert(risk_from_distance(.5, 1) == .5 &&
         risk_from_distance(std::nullopt, 1) == 0);
  std::filesystem::remove_all(root);
}
