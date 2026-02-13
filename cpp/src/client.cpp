#include "horus/client.hpp"

namespace horus {

Client::Client(std::string backend, bool auto_launch)
    : backend_type_(std::move(backend)), auto_launch_(auto_launch) {}

void Client::shutdown() {
    // No-op for current SDK scaffold.
}

} // namespace horus

