#include "horus/mapping/live_mesh.hpp"
#include <cassert>
#include <fstream>
#include <limits>
#include <nlohmann/json.hpp>

using namespace horus::mapping;
using Json = nlohmann::json;
template<class F> bool rejects(F&& fn) { try { fn(); } catch(const std::exception&) { return true; } return false; }
int main() {
    BlockGeometry g{{{0,0,0},{1,0,0},{0,1,0}},{{255,0,0,255},{0,255,0,255},{0,0,255,255}},{0,1,2}};
    LiveMeshState map({"map",2,120,240,11});
    auto first = map.replace("a",&g).at(0);
    assert(first->payload.size()==112);
    assert(map.replace("a",&g).empty());
    assert(map.unchanged_pages()==1 && map.suppressed_bytes()==112);
    map.replace("b",&g);
    assert(map.snapshot_updates()[0]==first);
    auto deleted = map.replace("a",nullptr).at(0);
    assert(deleted->operation==MeshOperation::Delete && deleted->revision==2);
    assert(map.cached_bytes()==164 && map.page_count()==1);
    auto added = map.replace("c",&g).at(0);
    assert(added->chunk_id==1 && added->revision==3);
    std::ifstream fixture(LIVE_MESH_FIXTURE);
    const auto expected = Json::parse(fixture);
    assert(Json::parse(map.manifest_json())==expected);
    auto repair = map.repair_updates(R"({"request":"pages","epoch":"11","ids":[1,1]})");
    assert(repair.size()==1 && repair[0]==added);
    assert(map.repair_updates(R"({"request":"pages","epoch":10})").size()==2);
    assert(map.repair_updates(R"({"request":"pages","epoch":11,"ids":[true]})").empty());
    assert(map.repair_updates(R"({"request":"pages","epoch":11,"ids":[1,2,1]})").empty());
    assert(map.repair_updates("malformed").empty());
    assert(map.clear()->epoch==12 && map.cached_bytes()==0);
    assert(map.repair_updates(R"({"request":"snapshot"})")[0]->operation==MeshOperation::Clear);
    assert(first->payload.size()==112);
    map.replace("a",&g);
    const auto before=map.manifest_json();
    auto invalid=g; invalid.triangles[0]=10;
    std::vector<std::pair<std::string,const BlockGeometry*>> changes{{"a",nullptr},{"bad",&invalid}};
    assert(rejects([&]{map.replace_many(changes);}));
    assert(map.manifest_json()==before);
    auto changed=g; changed.colors[0]={0,0,0,255};
    assert(map.replace("a",&changed)[0]->revision==2);
    map.replace("b",&g);
    const auto full=map.manifest_json();
    assert(rejects([&]{map.replace("c",&g);}));
    assert(map.manifest_json()==full);
    LiveMeshState limited({"map",2,120,120,11});
    limited.replace("a",&g);
    assert(rejects([&]{limited.replace("b",&g);}));
    assert(limited.page_count()==1 && limited.cached_bytes()==112);
    invalid.vertices[0][0]=std::numeric_limits<float>::quiet_NaN();
    assert(rejects([&]{limited.replace("a",&invalid);}));
    assert(rejects([]{LiveMeshState invalid({"map",0});}));
}
