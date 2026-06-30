#include "horus/experiments/field_teammate_study.hpp"
#include "horus/experiments/metrics.hpp"

#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

namespace hexp = horus::experiments;

namespace {

bool approx(double a, double b) {
    return std::fabs(a - b) < 1e-6;
}

std::string write_session(
    const std::string& dyad_id,
    hexp::StudyCondition condition,
    long long base_ns,
    double mission_s,
    int clarifications,
    int nav_errors,
    double guidance_latency_s) {
    const auto path = (std::filesystem::temp_directory_path() /
                       ("horus_cpp_ft_" + dyad_id + ".ndjson"))
                          .string();
    const long long latency_ns = static_cast<long long>(guidance_latency_s * 1e9);
    hexp::FieldTeammateStudyRecorder rec(path, dyad_id, condition, "alpha");
    rec.record(hexp::study_event::SESSION_START, {}, base_ns);
    for (int i = 0; i < clarifications; ++i) {
        const std::string rid = dyad_id + "-clar-" + std::to_string(i);
        rec.record(hexp::study_event::GUIDANCE_REQUEST, {{"request_id", rid}}, base_ns + 1000000);
        rec.record(
            hexp::study_event::GUIDANCE_RESPONSE,
            {{"request_id", rid}, {"outcome", std::string(hexp::guidance_outcome::CLARIFY)}},
            base_ns + 1000000 + latency_ns);
    }
    rec.record(hexp::study_event::GUIDANCE_REQUEST, {{"request_id", std::string("ack-1")}}, base_ns + 2000000);
    rec.record(
        hexp::study_event::GUIDANCE_RESPONSE,
        {{"request_id", std::string("ack-1")}, {"outcome", std::string(hexp::guidance_outcome::ACKNOWLEDGE)}},
        base_ns + 2000000 + latency_ns);
    for (int i = 0; i < nav_errors; ++i) {
        rec.record(hexp::study_event::NAVIGATION_ERROR, {}, base_ns + 3000000);
    }
    rec.record(hexp::study_event::COMMUNICATION, {{"duration_s", 2.0}}, base_ns + 4000000);
    rec.record(
        hexp::study_event::LOCALIZATION_SPOTCHECK,
        {{"error_m", 0.12}, {"heading_error_deg", 3.0}},
        base_ns + 5000000);
    rec.record(hexp::study_event::TASK_SUCCESS, {}, base_ns + 6000000);
    rec.record(hexp::study_event::SESSION_END, {}, base_ns + static_cast<long long>(mission_s * 1e9));
    rec.flush();
    return path;
}

void test_ndjson_round_trip() {
    hexp::JsonObject obj{
        {"event_type", std::string("communication")},
        {"count", static_cast<long long>(7)},
        {"duration_s", 1.5},
        {"flag", true},
        {"note", std::string("a,\"b\"\n")},
    };
    const auto line = hexp::serialize_ndjson_object(obj);
    const auto parsed = hexp::parse_ndjson_object(line);
    assert(hexp::field_string(parsed, "event_type") == "communication");
    assert(hexp::field_ll(parsed, "count") == 7);
    assert(approx(hexp::field_double(parsed, "duration_s"), 1.5));
    assert(parsed.at("flag") == hexp::JsonField{true});
    assert(hexp::field_string(parsed, "note") == "a,\"b\"\n");
}

void test_session_metrics() {
    const auto path = write_session("dyad1", hexp::StudyCondition::InTheTeam, 1000000000LL, 120.0, 2, 1, 0.5);
    const auto session = hexp::SessionRecord::from_ndjson(path);
    const auto metrics = hexp::session_metrics(session);

    assert(approx(metrics.at("mission_completion_time_s"), 120.0));
    assert(approx(metrics.at("clarification_requests"), 2.0));
    assert(approx(metrics.at("navigation_errors"), 1.0));
    assert(approx(metrics.at("guidance_requests"), 3.0));
    assert(approx(metrics.at("communication_count"), 1.0));
    assert(approx(metrics.at("task_success_rate"), 1.0));
    assert(approx(metrics.at("mean_guidance_latency_s"), 0.5));
    assert(approx(metrics.at("mean_localization_error_m"), 0.12));
    std::filesystem::remove(path);
}

void test_aggregate_dyad_unit() {
    std::vector<hexp::SessionRecord> sessions;
    const double missions_map[] = {100.0, 110.0, 120.0};
    for (int i = 0; i < 3; ++i) {
        const auto path = write_session("dyadmap" + std::to_string(i), hexp::StudyCondition::OnTheMap,
                                        1000000000LL, missions_map[i], 3, 2, 0.8);
        sessions.push_back(hexp::SessionRecord::from_ndjson(path));
        std::filesystem::remove(path);
    }
    const double missions_team[] = {70.0, 80.0};
    for (int i = 0; i < 2; ++i) {
        const auto path = write_session("dyadteam" + std::to_string(i), hexp::StudyCondition::InTheTeam,
                                        1000000000LL, missions_team[i], 1, 0, 0.3);
        sessions.push_back(hexp::SessionRecord::from_ndjson(path));
        std::filesystem::remove(path);
    }

    const auto agg = hexp::aggregate_sessions(sessions);
    const auto& on_map = agg.at("c2_on_the_map").at("mission_completion_time_s");
    assert(on_map.n == 3);
    assert(approx(on_map.mean, 110.0));
    assert(on_map.ci95_low < on_map.mean && on_map.mean < on_map.ci95_high);

    const auto& in_team = agg.at("c3_in_the_team").at("mission_completion_time_s");
    assert(in_team.n == 2);
    assert(approx(in_team.mean, 75.0));
    assert(approx(agg.at("c3_in_the_team").at("clarification_requests").mean, 1.0));
    assert(approx(agg.at("c2_on_the_map").at("clarification_requests").mean, 3.0));
}

void test_summarize_values() {
    const auto s = hexp::summarize_values("x", "c", {10.0, 12.0, 14.0, 16.0, 18.0});
    assert(s.n == 5);
    assert(approx(s.mean, 14.0));
    assert(s.ci95_low < 14.0 && 14.0 < s.ci95_high);
    const auto single = hexp::summarize_values("x", "c", {5.0});
    assert(approx(single.ci95_low, 5.0) && approx(single.ci95_high, 5.0));
}

}  // namespace

int main() {
    test_ndjson_round_trip();
    test_session_metrics();
    test_aggregate_dyad_unit();
    test_summarize_values();
    std::cout << "cpp_experiments_tests passed" << std::endl;
    return 0;
}
