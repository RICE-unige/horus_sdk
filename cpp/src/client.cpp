#include "horus/client.hpp"
#include "horus/bridge/robot_registry.hpp"
#include <stdexcept>

namespace horus {

Client::Client(std::string backend, bool auto_launch)
    : backend_type_(std::move(backend)), auto_launch_(auto_launch),
      registry_(std::make_shared<bridge::RobotRegistryClient>(bridge::RegistryOptions{.auto_launch=auto_launch})) {}

bridge::RobotRegistryClient& Client::registry() {
    if (backend_type_ != "ros2") throw std::invalid_argument("Native registration requires backend=ros2");
    return *registry_;
}

void Client::shutdown() {
    registry_->stop();
}

} // namespace horus
