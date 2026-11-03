#include "horus/mapping/costmap.hpp"
#include "horus/mapping/mesh_protocol.hpp"
#include <chrono>
#include <iostream>
#include <numeric>

int main() {
    using namespace horus::mapping;
    using Clock = std::chrono::steady_clock;
    CostmapState map;
    map.full({"map",1024,1024,0.05},std::vector<std::int8_t>(1024*1024),0);
    std::vector<std::uint8_t> patch(128*128);
    for (std::size_t i=0; i<patch.size(); ++i) patch[i]=static_cast<std::uint8_t>(i%256);
    BlockGeometry geometry;
    for (unsigned i=0; i<30000; ++i) {
        geometry.vertices.push_back({static_cast<float>(i%256)/16,static_cast<float>(i/256)/16,1});
        geometry.colors.push_back({255,128,0,255});
        geometry.triangles.push_back(i);
    }
    std::uint64_t stamp=0;
    for (int round=0; round<6; ++round) {
        auto begin = Clock::now();
        for (int i=0; i<1000; ++i) map.patch_raw("map",64,64,128,128,patch,++stamp);
        const auto patch_us = std::chrono::duration<double,std::micro>(Clock::now()-begin).count()/1000;
        std::vector<std::uint8_t> encoded;
        begin = Clock::now();
        for (int i=0; i<100; ++i) encoded=encode_live_payload(7,{0,0,0},1,1,MeshOperation::Upsert,&geometry,1048576);
        const auto encode_us = std::chrono::duration<double,std::micro>(Clock::now()-begin).count()/100;
        if (round) std::cout << "{\"language\":\"cpp\",\"round\":" << round
            << ",\"patch_us\":" << patch_us << ",\"encode_us\":" << encode_us
            << ",\"wire_bytes\":" << encoded.size() << ",\"checksum\":"
            << std::accumulate(encoded.begin(),encoded.end(),std::uint64_t{}) << "}\n";
    }
}
