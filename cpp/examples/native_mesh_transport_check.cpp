#include "horus/mapping/live_mesh_publisher.hpp"
#include <chrono>
#include <iostream>
#include <thread>

// Isolated ROS contract fixture, not a robot demo or registration launcher.
int main(int argc, char** argv) {
    if (argc != 2) { std::cerr << "Usage: native_mesh_transport_check /absolute/map/topic\n"; return 2; }
    try {
        using namespace horus::mapping;
        LiveMeshPublisherOptions options;
        options.topic = argv[1];
        options.map.initial_epoch = 11;
        LiveMeshPublisher publisher(options);
        BlockGeometry triangle{{{0,0,0},{1,0,0},{0,1,0}},{{255,0,0,255},{0,255,0,255},{0,0,255,255}},{0,1,2}};
        if (publisher.replace("triangle", &triangle) != 1 || publisher.replace("triangle", &triangle) != 0)
            throw std::runtime_error("Mesh deduplication failed");
        auto start = std::chrono::steady_clock::now();
        int phase = 0;
        while (true) {
            const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
            if (elapsed >= 8.0) break;
            if (elapsed >= 4.0 && phase == 0) { triangle.colors[0] = {255,255,255,255}; publisher.replace("triangle", &triangle); ++phase; }
            if (elapsed >= 5.0 && phase == 1) { publisher.replace("triangle", nullptr); ++phase; }
            if (elapsed >= 6.0 && phase == 2) { publisher.clear(); ++phase; }
            publisher.pump();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        if (publisher.state().page_count() != 0 || publisher.pending_count() != 0 || publisher.dropped_repair_requests() != 0)
            throw std::runtime_error("Mesh lifecycle or repair overflow check failed");
        std::cout << "Native mesh lifecycle passed\n";
    } catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
