#ifndef HORUS_EXPERIMENTS_FIELD_TEAMMATE_STUDY_HPP
#define HORUS_EXPERIMENTS_FIELD_TEAMMATE_STUDY_HPP

#include "horus/experiments/metrics.hpp"

#include <array>
#include <map>
#include <optional>
#include <string>
#include <vector>

namespace horus {
namespace experiments {

// "On the Map, In the Team" study conditions (additive ladder).
enum class StudyCondition { VoiceVideo, OnTheMap, InTheTeam };

std::string study_condition_to_string(StudyCondition condition);
std::optional<StudyCondition> coerce_study_condition(const std::string& value);
std::array<StudyCondition, 3> study_condition_ladder();

// Event-type tokens (match the Python/Rust schema).
namespace study_event {
inline constexpr const char* SESSION_START = "session_start";
inline constexpr const char* SESSION_END = "session_end";
inline constexpr const char* GUIDANCE_REQUEST = "guidance_request";
inline constexpr const char* GUIDANCE_RESPONSE = "guidance_response";
inline constexpr const char* CLARIFICATION = "clarification";
inline constexpr const char* COMMUNICATION = "communication";
inline constexpr const char* UNCERTAINTY_RAISED = "uncertainty_raised";
inline constexpr const char* UNCERTAINTY_RESOLVED = "uncertainty_resolved";
inline constexpr const char* NAVIGATION_ERROR = "navigation_error";
inline constexpr const char* WAYPOINT_REACHED = "waypoint_reached";
inline constexpr const char* TASK_SUCCESS = "task_success";
inline constexpr const char* TASK_FAILURE = "task_failure";
inline constexpr const char* SAFETY_EVENT = "safety_event";
inline constexpr const char* LOCALIZATION_SPOTCHECK = "localization_spotcheck";
}  // namespace study_event

namespace guidance_outcome {
inline constexpr const char* ACKNOWLEDGE = "acknowledge";
inline constexpr const char* CLARIFY = "clarify";
inline constexpr const char* REJECT = "reject";
inline constexpr const char* COMPLETE = "complete";
}  // namespace guidance_outcome

// Append study events for one dyad/session to an NDJSON log.
class FieldTeammateStudyRecorder {
public:
    FieldTeammateStudyRecorder(
        const std::string& path,
        std::string dyad_id,
        StudyCondition condition,
        std::string scenario);

    void record(const std::string& event_type, JsonObject payload = {}, std::optional<long long> timestamp_ns = std::nullopt);
    void flush();

private:
    NdjsonEventWriter writer_;
    std::string dyad_id_;
    std::string scenario_;
};

struct SessionRecord {
    std::string dyad_id;
    StudyCondition condition{StudyCondition::VoiceVideo};
    std::string scenario;
    std::vector<JsonObject> events;

    static SessionRecord from_ndjson(const std::string& path);
};

// Reduce a single dyad session to its dependent variables.
std::map<std::string, double> session_metrics(const SessionRecord& session);

struct MetricSummary {
    std::string metric;
    std::string condition;
    int n{0};
    double mean{0.0};
    double sd{0.0};
    double sem{0.0};
    double ci95_low{0.0};
    double ci95_high{0.0};
};

MetricSummary summarize_values(
    const std::string& metric,
    const std::string& condition,
    const std::vector<double>& values);

// Aggregate per-dyad session metrics into per-condition summaries (dyad unit).
std::map<std::string, std::map<std::string, MetricSummary>> aggregate_sessions(
    const std::vector<SessionRecord>& sessions);

}  // namespace experiments
}  // namespace horus

#endif  // HORUS_EXPERIMENTS_FIELD_TEAMMATE_STUDY_HPP
