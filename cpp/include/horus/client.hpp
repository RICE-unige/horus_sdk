#ifndef HORUS_CLIENT_HPP
#define HORUS_CLIENT_HPP

#include <string>

namespace horus {

class Client {
public:
    explicit Client(std::string backend = "ros2", bool auto_launch = true);

    const std::string& backend_type() const { return backend_type_; }
    bool auto_launch() const { return auto_launch_; }

    void shutdown();

private:
    std::string backend_type_;
    bool auto_launch_{true};
};

} // namespace horus

#endif // HORUS_CLIENT_HPP

