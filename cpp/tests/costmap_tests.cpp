#include "horus/mapping/costmap.hpp"
#include <iostream>
#include <limits>
#include <stdexcept>

using namespace horus::mapping;
void require(bool condition) { if (!condition) throw std::runtime_error("Costmap contract failed"); }
template<class F> void rejects(F&& fn) {
    bool failed=false;
    try { fn(); } catch (const std::invalid_argument&) { failed=true; }
    require(failed);
}
int main() {
    CostmapState state;
    GridGeometry geometry{"map",3,2,.05};
    const std::vector<std::uint8_t> raw{0,1,252,253,254,255};
    require(state.full_raw(geometry,raw,10));
    auto equal = [&](std::vector<std::int8_t> expected) {
        require(std::equal(state.data().begin(),state.data().end(),expected.begin(),expected.end()));
    };
    equal({0,1,98,99,100,-1});
    require(state.patch(" /map ",1,0,1,2,std::vector<std::int8_t>{40,50},11));
    equal({0,40,98,99,50,-1});
    require(!state.patch("map",0,0,1,1,std::vector<std::int8_t>{1},9));
    rejects([&]{state.patch("map",0,0,2,1,std::vector<std::int8_t>{5,101},12);});
    rejects([&]{state.patch("map",UINT32_MAX,0,2,1,std::vector<std::int8_t>{5,5},12);});
    rejects([&]{state.patch("other",0,0,1,1,std::vector<std::int8_t>{5},12);});
    equal({0,40,98,99,50,-1});
    require(state.revision()==2 && state.stamp_ns()==11);
    for (const auto resolution : {0.,-1.,std::numeric_limits<double>::infinity(),std::numeric_limits<double>::quiet_NaN(),std::numeric_limits<double>::max()}) {
        auto invalid = geometry; invalid.resolution = resolution;
        rejects([&]{state.full_raw(invalid,raw,12);});
    }
    auto invalid = geometry; invalid.origin[6]=0;
    rejects([&]{state.full_raw(invalid,raw,12);});
    equal({0,40,98,99,50,-1});
    std::cout << "Native costmap atomicity and encoding checks passed\n";
    state.full(geometry,{1,2,3,4,5,6},20);
    state.patch("map",1,0,2,2,state.data().first(4),21);
    equal({1,1,2,4,3,4});
}
