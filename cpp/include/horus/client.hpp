#ifndef HORUS_CLIENT_HPP
#define HORUS_CLIENT_HPP

#include <string>
#include <memory>

namespace horus {
namespace bridge { class RobotRegistryClient; }

class Client {
public:
    explicit Client(std::string backend = "ros2", bool auto_launch = true);

    const std::string& backend_type() const { return backend_type_; }
    bool auto_launch() const { return auto_launch_; }

    // This client's registry is independent from the global convenience registry.
    bridge::RobotRegistryClient& registry();

    void shutdown();

private:
    std::string backend_type_;
    bool auto_launch_{true};
    std::shared_ptr<bridge::RobotRegistryClient> registry_;
};

} // namespace horus

#endif // HORUS_CLIENT_HPP
