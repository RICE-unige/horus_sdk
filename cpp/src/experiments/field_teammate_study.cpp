#include "horus/experiments/field_teammate_study.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <numeric>
#include <set>
#include <stdexcept>

namespace horus {
namespace experiments {

namespace {

constexpr double kNsPerS = 1e9;

long long event_ts(const JsonObject& event) {
    return field_ll(event, "timestamp_ns", 0);
}

std::vector<const JsonObject*> of_type(const SessionRecord& session, const std::string& kind) {
    std::vector<const JsonObject*> out;
    for (const auto& event : session.events) {
        if (field_string(event, "event_type") == kind) {
            out.push_back(&event);
        }
    }
    return out;
}

double mean(const std::vector<double>& values) {
    if (values.empty()) {
        return std::nan("");
    }
    return std::accumulate(values.begin(), values.end(), 0.0) / static_cast<double>(values.size());
}

double sample_stdev(const std::vector<double>& values, double m) {
    if (values.size() < 2) {
        return 0.0;
    }
    double acc = 0.0;
    for (const double v : values) {
        acc += (v - m) * (v - m);
    }
    return std::sqrt(acc / static_cast<double>(values.size() - 1));
}

double t_critical_95(int df) {
    static const std::array<double, 30> table{
        12.706, 4.303, 3.182, 2.776, 2.571, 2.447, 2.365, 2.306, 2.262, 2.228, 2.201, 2.179,
        2.160, 2.145, 2.131, 2.120, 2.110, 2.101, 2.093, 2.086, 2.080, 2.074, 2.069, 2.064,
        2.060, 2.056, 2.052, 2.048, 2.045, 2.042};
    if (df <= 0) {
        return std::nan("");
    }
    if (df <= 30) {
        return table[static_cast<std::size_t>(df - 1)];
    }
    return 1.96;
}

}  // namespace

std::string study_condition_to_string(StudyCondition condition) {
    switch (condition) {
        case StudyCondition::VoiceVideo: return "c1_voice_video";
        case StudyCondition::OnTheMap: return "c2_on_the_map";
        case StudyCondition::InTheTeam: return "c3_in_the_team";
    }
    return "c1_voice_video";
}

std::optional<StudyCondition> coerce_study_condition(const std::string& value) {
    std::string v;
    v.reserve(value.size());
    for (const char c : value) {
        if (c != ' ' && c != '\t') {
            v.push_back(static_cast<char>(std::tolower(c)));
        }
    }
    if (v == "c1_voice_video" || v == "voicevideo" || v == "voice_video") {
        return StudyCondition::VoiceVideo;
    }
    if (v == "c2_on_the_map" || v == "onthemap" || v == "on_the_map") {
        return StudyCondition::OnTheMap;
    }
    if (v == "c3_in_the_team" || v == "intheteam" || v == "in_the_team") {
        return StudyCondition::InTheTeam;
    }
    return std::nullopt;
}

std::array<StudyCondition, 3> study_condition_ladder() {
    return {StudyCondition::VoiceVideo, StudyCondition::OnTheMap, StudyCondition::InTheTeam};
}

FieldTeammateStudyRecorder::FieldTeammateStudyRecorder(
    const std::string& path,
    std::string dyad_id,
    StudyCondition condition,
    std::string scenario)
    : writer_(path, dyad_id, "field_teammate_study", study_condition_to_string(condition), "study"),
      dyad_id_(std::move(dyad_id)),
      scenario_(std::move(scenario)) {}

void FieldTeammateStudyRecorder::record(
    const std::string& event_type,
    JsonObject payload,
    std::optional<long long> timestamp_ns) {
    payload["event_type"] = event_type;
    payload["dyad_id"] = dyad_id_;
    payload["scenario"] = scenario_;
    writer_.write(std::move(payload), timestamp_ns);
}

void FieldTeammateStudyRecorder::flush() {
    writer_.flush();
}

SessionRecord SessionRecord::from_ndjson(const std::string& path) {
    std::ifstream in(path);
    if (!in) {
        throw std::runtime_error("failed to open session log: " + path);
    }
    SessionRecord record;
    std::string line;
    while (std::getline(in, line)) {
        if (line.find_first_not_of(" \t\r\n") == std::string::npos) {
            continue;
        }
        record.events.push_back(parse_ndjson_object(line));
    }
    if (record.events.empty()) {
        throw std::runtime_error("no events in session log: " + path);
    }
    std::stable_sort(record.events.begin(), record.events.end(),
                     [](const JsonObject& a, const JsonObject& b) { return event_ts(a) < event_ts(b); });
    const JsonObject& first = record.events.front();
    record.dyad_id = field_string(first, "dyad_id");
    record.scenario = field_string(first, "scenario");
    const auto condition = coerce_study_condition(field_string(first, "condition"));
    if (!condition) {
        throw std::runtime_error("unknown study condition in session log");
    }
    record.condition = *condition;
    return record;
}

std::map<std::string, double> session_metrics(const SessionRecord& session) {
    std::map<std::string, double> metrics;

    const auto starts = of_type(session, study_event::SESSION_START);
    const auto ends = of_type(session, study_event::SESSION_END);
    if (!starts.empty() && !ends.empty()) {
        metrics["mission_completion_time_s"] =
            static_cast<double>(event_ts(*ends.back()) - event_ts(*starts.front())) / kNsPerS;
    }

    const auto successes = of_type(session, study_event::TASK_SUCCESS).size();
    const auto failures = of_type(session, study_event::TASK_FAILURE).size();
    if (successes + failures > 0) {
        metrics["task_success_rate"] =
            static_cast<double>(successes) / static_cast<double>(successes + failures);
    }
    metrics["navigation_errors"] = static_cast<double>(of_type(session, study_event::NAVIGATION_ERROR).size());
    metrics["safety_events"] = static_cast<double>(of_type(session, study_event::SAFETY_EVENT).size());

    const auto responses = of_type(session, study_event::GUIDANCE_RESPONSE);
    std::size_t clarify_responses = 0;
    std::size_t reject_responses = 0;
    for (const auto* resp : responses) {
        const auto outcome = field_string(*resp, "outcome");
        if (outcome == guidance_outcome::CLARIFY) {
            ++clarify_responses;
        } else if (outcome == guidance_outcome::REJECT) {
            ++reject_responses;
        }
    }
    metrics["clarification_requests"] =
        static_cast<double>(of_type(session, study_event::CLARIFICATION).size() + clarify_responses);
    metrics["guidance_requests"] = static_cast<double>(of_type(session, study_event::GUIDANCE_REQUEST).size());
    metrics["guidance_rejections"] = static_cast<double>(reject_responses);

    const auto comms = of_type(session, study_event::COMMUNICATION);
    metrics["communication_count"] = static_cast<double>(comms.size());
    double comm_duration = 0.0;
    for (const auto* c : comms) {
        comm_duration += field_double(*c, "duration_s");
    }
    metrics["communication_total_duration_s"] = comm_duration;

    std::map<std::string, long long> request_times;
    for (const auto* req : of_type(session, study_event::GUIDANCE_REQUEST)) {
        const auto rid = field_string(*req, "request_id");
        if (!rid.empty()) {
            request_times[rid] = event_ts(*req);
        }
    }
    std::vector<double> latencies;
    for (const auto* resp : responses) {
        const auto rid = field_string(*resp, "request_id");
        const auto it = request_times.find(rid);
        if (it != request_times.end()) {
            latencies.push_back(static_cast<double>(event_ts(*resp) - it->second) / kNsPerS);
        }
    }
    if (!latencies.empty()) {
        metrics["mean_guidance_latency_s"] = mean(latencies);
    }

    std::map<std::string, long long> raised;
    for (const auto* ev : of_type(session, study_event::UNCERTAINTY_RAISED)) {
        const auto uid = field_string(*ev, "uncertainty_id");
        if (!uid.empty()) {
            raised[uid] = event_ts(*ev);
        }
    }
    std::vector<double> resolutions;
    for (const auto* ev : of_type(session, study_event::UNCERTAINTY_RESOLVED)) {
        const auto uid = field_string(*ev, "uncertainty_id");
        const auto it = raised.find(uid);
        if (it != raised.end()) {
            resolutions.push_back(static_cast<double>(event_ts(*ev) - it->second) / kNsPerS);
        }
    }
    if (!resolutions.empty()) {
        metrics["mean_uncertainty_resolution_s"] = mean(resolutions);
    }

    const auto spotchecks = of_type(session, study_event::LOCALIZATION_SPOTCHECK);
    if (!spotchecks.empty()) {
        std::vector<double> errors;
        std::vector<double> headings;
        for (const auto* s : spotchecks) {
            errors.push_back(field_double(*s, "error_m"));
            headings.push_back(field_double(*s, "heading_error_deg"));
        }
        metrics["mean_localization_error_m"] = mean(errors);
        metrics["mean_heading_error_deg"] = mean(headings);
    }

    return metrics;
}

MetricSummary summarize_values(
    const std::string& metric,
    const std::string& condition,
    const std::vector<double>& values) {
    std::vector<double> clean;
    clean.reserve(values.size());
    for (const double v : values) {
        if (!std::isnan(v)) {
            clean.push_back(v);
        }
    }
    MetricSummary summary;
    summary.metric = metric;
    summary.condition = condition;
    summary.n = static_cast<int>(clean.size());
    if (clean.empty()) {
        const double nan = std::nan("");
        summary.mean = summary.sd = summary.sem = summary.ci95_low = summary.ci95_high = nan;
        return summary;
    }
    const double m = mean(clean);
    summary.mean = m;
    if (clean.size() == 1) {
        summary.ci95_low = summary.ci95_high = m;
        return summary;
    }
    summary.sd = sample_stdev(clean, m);
    summary.sem = summary.sd / std::sqrt(static_cast<double>(clean.size()));
    const double margin = t_critical_95(summary.n - 1) * summary.sem;
    summary.ci95_low = m - margin;
    summary.ci95_high = m + margin;
    return summary;
}

std::map<std::string, std::map<std::string, MetricSummary>> aggregate_sessions(
    const std::vector<SessionRecord>& sessions) {
    std::map<std::string, std::vector<std::map<std::string, double>>> by_condition;
    for (const auto& session : sessions) {
        by_condition[study_condition_to_string(session.condition)].push_back(session_metrics(session));
    }

    std::map<std::string, std::map<std::string, MetricSummary>> result;
    for (const auto& [condition, dyad_metrics] : by_condition) {
        std::set<std::string> metric_names;
        for (const auto& metrics : dyad_metrics) {
            for (const auto& [name, _] : metrics) {
                metric_names.insert(name);
            }
        }
        std::map<std::string, MetricSummary> summaries;
        for (const auto& name : metric_names) {
            std::vector<double> values;
            for (const auto& metrics : dyad_metrics) {
                const auto it = metrics.find(name);
                if (it != metrics.end()) {
                    values.push_back(it->second);
                }
            }
            summaries[name] = summarize_values(name, condition, values);
        }
        result[condition] = std::move(summaries);
    }
    return result;
}

}  // namespace experiments
}  // namespace horus
