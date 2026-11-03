#include "horus/mapping/live_mesh_publisher.hpp"
#include <stdexcept>
#include <string>
#if defined(HORUS_ROS2_ENABLED)
#include <rclcpp/rclcpp.hpp>
#include "mesh_transport.h"
#include "registry.h"
#endif

void check(bool result) { if (!result) throw std::runtime_error("Mesh publisher lifecycle check failed"); }

int main() {
    using namespace horus::mapping;
#if defined(HORUS_ROS2_ENABLED)
    rclcpp::init(0, nullptr);
    {
        LiveMeshPublisher publisher({.topic="/horus_test_mesh_lifecycle"});
        BlockGeometry geometry{{{0,0,0},{1,0,0},{0,1,0}},{},{0,1,2}};
        check(publisher.replace("a", &geometry) == 1);
        check(publisher.replace("a", &geometry) == 0);
        check(publisher.pump() == 0 && publisher.pending_count() == 1);
        check(publisher.replace("a", nullptr) == 1);
        publisher.clear();
        check(publisher.state().page_count() == 0 && publisher.pending_count() == 1);
        publisher.pump();
    }
    check(rclcpp::ok());
    for (int i = 0; i < 20; ++i) {
        char* failure = nullptr;
        auto* immediate = horus_native_mesh_create("/horus_test_mesh_immediate", 1024, &failure);
        check(immediate && !failure);
        horus_native_mesh_destroy(immediate);
        auto* registry = horus_native_create_with_options(R"({"auto_launch":false})", &failure);
        check(registry && !failure);
        horus_native_destroy(registry);
    }
    check(rclcpp::ok());
    for (const auto* topic : {"relative", "/trailing/", "/invalid space", "/{substitution}"}) {
        char* error = nullptr;
        void* handle = horus_native_mesh_create(topic, 1024, &error);
        check(handle == nullptr && error != nullptr);
        horus_native_free(error);
    }
    char* error = nullptr;
    check(horus_native_mesh_publish(nullptr, nullptr, 0, &error) == 0 && error != nullptr);
    horus_native_free(error);
    error = nullptr;
    void* handle = horus_native_mesh_create("/horus_test_mesh_invalid", 1024, &error);
    check(handle && !error);
    check(horus_native_mesh_publish(handle, nullptr, 1024, &error) == 0 && error);
    horus_native_free(error);
    error = nullptr;
    check(horus_native_mesh_manifest(handle, "[1]", &error) == 0 && error);
    horus_native_free(error);
    horus_native_mesh_destroy(handle);
    check(rclcpp::ok());
    rclcpp::shutdown();
#else
    bool rejected = false;
    try { LiveMeshPublisher publisher; }
    catch (const std::runtime_error& error) { rejected = std::string(error.what()).find("ROS 2") != std::string::npos; }
    check(rejected);
#endif
}
