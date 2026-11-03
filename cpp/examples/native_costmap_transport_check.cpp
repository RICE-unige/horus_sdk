#include "horus/mapping/costmap_adapter.hpp"
#include <chrono>
#include <iostream>
#include <thread>
int main(int argc,char** argv) {
    try {
        horus::mapping::CostmapAdapterOptions options;
        options.input_topic="/horus_native_costmap/full"; options.output_topic="/horus_native_costmap/output";
        if(argc>1 && std::string(argv[1])=="raw") options.input_format="raw";
        options.updates_topic="/horus_native_costmap/updates"; options.snapshot_service="/horus_native_costmap/snapshot";
        options.publish_hz=20; options.resync_seconds=0.5;
        horus::mapping::CostmapAdapter adapter(options);
        for(int i=0;i<160;++i) {
            if(!adapter.status().at("running").get<bool>()) throw std::runtime_error(adapter.status().dump());
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        std::cout<<adapter.status().dump()<<'\n'; return 0;
    } catch(const std::exception& e) { std::cerr<<e.what()<<'\n'; return 1; }
}
