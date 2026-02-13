#ifndef HORUS_BRIDGE_UNITY_TCP_HPP
#define HORUS_BRIDGE_UNITY_TCP_HPP

#include <string>

namespace horus {
namespace bridge {

class UnityTcpBridge {
public:
    explicit UnityTcpBridge(int port = 10000) : port_(port) {}

    int port() const { return port_; }
    bool connected() const { return connected_; }
    void set_connected(bool connected) { connected_ = connected; }

private:
    int port_{10000};
    bool connected_{false};
};

} // namespace bridge
} // namespace horus

#endif // HORUS_BRIDGE_UNITY_TCP_HPP

