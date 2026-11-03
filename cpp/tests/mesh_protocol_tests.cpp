#include "horus/mapping/mesh_protocol.hpp"
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>

using namespace horus::mapping;
void require(bool condition) { if (!condition) throw std::runtime_error("HMSH contract failed"); }
int main() {
    BlockGeometry g{{{0,0,0},{1,0,0},{0,1,0}},{{255,0,0,255},{0,255,0,255},{0,0,255,255}},{0,1,2}};
    const auto encoded = encode_live_payload(7,{-2,3,0},11,12,MeshOperation::Upsert,&g,1024);
    std::ostringstream hex;
    for (auto byte : encoded) hex << std::hex << std::setfill('0') << std::setw(2) << static_cast<unsigned>(byte);
    std::ifstream fixture(HORUS_MESH_FIXTURE);
    std::string expected; fixture >> expected;
    require(hex.str()==expected);
    const auto header=decode_live_header(encoded);
    require(header.page_key[0]==-2 && header.has_color && header.index_count==3 && header.epoch==11);
    const auto clear=encode_live_payload(0,{0,0,0},1,0,MeshOperation::Clear,nullptr,52);
    require(clear.size()==52 && decode_live_header(clear).operation==MeshOperation::Clear);
    bool rejected=false;
    try { encode_live_payload(0,{0,0,0},1,0,MeshOperation::Upsert,&g,100); }
    catch (const std::invalid_argument&) { rejected=true; }
    require(rejected);
    g.triangles[0]=3; rejected=false;
    try { encode_live_payload(0,{0,0,0},1,0,MeshOperation::Upsert,&g,1024); }
    catch (const std::invalid_argument&) { rejected=true; }
    require(rejected);
}
