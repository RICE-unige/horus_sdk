#include "horus/mapping/mesh_source_adapter.hpp"
#include <chrono>
#include <iostream>
#include <thread>
int main(int argc, char **argv) {
    try {
        if (argc < 4)
            return 1;
        horus::mapping::MeshSourceAdapterOptions o;
        o.input_type = argv[1];
        o.input_topic = argv[2];
        o.output_topic = argv[3];
        o.source.triangles_per_page = 1;
        o.source.blocks_per_axis = 1;
        horus::mapping::MeshSourceAdapter adapter(o);
        std::this_thread::sleep_for(std::chrono::seconds(12));
        std::cout << adapter.status().dump() << std::endl;
    } catch (const std::exception &e) {
        std::cerr << e.what() << '\n';
        return 2;
    }
}
