#pragma once
#include <string>
#include <vector>

// A directly spawned bridge belongs to this registry only. Existing bridges
// are never adopted or stopped. All methods are serialized by the caller.
class BridgeProcess {
public:
    ~BridgeProcess();
    BridgeProcess() = default;
    BridgeProcess(const BridgeProcess&) = delete;
    BridgeProcess& operator=(const BridgeProcess&) = delete;
    void start(const std::string& executable, const std::vector<std::string>& arguments);
    void poll();
    void stop() noexcept;
    bool started() const { return started_; }
    bool running() const { return pid_ > 0; }
    const std::string& error() const { return error_; }
    const std::string& log_tail() const { return tail_; }
    static std::string locate();
private:
    void drain();
    int pid_{-1}, output_{-1};
    bool started_{false};
    std::string tail_, error_;
};
