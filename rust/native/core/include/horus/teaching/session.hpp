#pragma once
#include "recording.hpp"
#include <functional>
#include <memory>

namespace horus::teaching {
struct AdapterCapabilities {
    std::string adapter_id = "observe", qualification = "observation";
    std::vector<std::string> joint_names;
    std::string command_frame, calibration_id;
    std::string action_representation = "none", controller_watchdog = "unavailable",
                profile_revision = "unspecified";
    Json joint_units = Json::object();
    std::string command_boundary = "unavailable", hold_description = "unavailable";
    std::optional<double> command_period_seconds;
    std::string authority_contract = "unavailable";
    bool supports_tracking_pause = false;
    Json to_dict() const;
    static AdapterCapabilities from_dict(const Json &);
};

class TeachAdapter {
  public:
    virtual ~TeachAdapter() = default;
    virtual AdapterCapabilities capabilities() const = 0;
    virtual std::pair<bool, std::string> ready() = 0;
    virtual void anchor(const Json &sample) = 0;
    virtual Json apply(const Json &sample) = 0;
    virtual bool hold(const Json &context) = 0;
    // Confirm a measured hold without revoking authority or changing the anchor.
    // Implementations must discard pending targets; a subsequent fresh apply resumes.
    virtual bool pause(const Json &) { throw std::runtime_error("Tracking pause unavailable"); }
};
class ObserveAdapter final : public TeachAdapter {
  public:
    AdapterCapabilities capabilities() const override { return {}; }
    std::pair<bool, std::string> ready() override {
        return {false, "Observation-only adapter; physical control is unavailable"};
    }
    void anchor(const Json &) override;
    Json apply(const Json &) override;
    bool hold(const Json &) override { return false; }
};

// Control calls are serialized; all disk work runs on the recorder/IO workers.
// Robot adapters must be bounded, nonblocking, and enforce authority themselves.
class TeachSession {
  public:
    TeachSession(TeachConfig, std::string robot_name, std::shared_ptr<TeachAdapter> adapter = {},
                 std::function<double()> clock = monotonic_seconds);
    ~TeachSession();
    TeachSession(const TeachSession &) = delete;
    TeachSession &operator=(const TeachSession &) = delete;
    void update_lease(const std::string &owner, const std::string &generation, double ttl);
    void note_stream(const std::string &stream_id);
    std::vector<std::string> missing_streams(bool control = false) const;
    std::pair<bool, std::string> readiness() const;
    Json state(const std::string &request_id = "", bool accepted = true,
               const std::string &reason = "");
    Json request(const Json &);
    bool input(const Json &);
    bool append(Sample, const std::vector<std::string> &cache_keys = {"latest"});
    bool recording() const;
    void tick();
    void close();

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
} // namespace horus::teaching
