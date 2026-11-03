#include "mesh_source_api.h"
#include "api.h"
#include <cassert>
#include <cmath>
#include <limits>
#include <nlohmann/json.hpp>
int main() {
    char* error=nullptr;
    auto* h=horus_mesh_source_create("{\"initial_epoch\":42}",&error);assert(h && !error);
    double points[]{0,0,0,1,0,0,0,1,0};
    horus_triangle_marker m{};m.name_space="room";m.frame="map";m.type=11;
    m.pose.rotation[3]=m.transform.rotation[3]=1;m.scale[0]=m.scale[1]=m.scale[2]=1;
    m.color[0]=10;m.color[1]=20;m.color[2]=30;m.color[3]=255;m.points=points;m.point_count=3;
    auto* batch=horus_mesh_source_markers(h,&m,1,0,&error);assert(batch && !error);
    assert(horus_mesh_batch_size(batch)==1);size_t bytes=0;const auto* p=horus_mesh_batch_page(batch,0,&bytes);assert(p && bytes>52);
    horus_mesh_batch_destroy(batch);
    points[0]=std::numeric_limits<double>::infinity();
    batch=horus_mesh_source_markers(h,&m,1,0,&error);assert(!batch && error);horus_teach_free(error);
    auto* status=horus_mesh_source_status(h,&error);assert(status && !error);
    assert(nlohmann::json::parse(status)["page_count"]==1);horus_teach_free(status);
    batch=horus_mesh_source_pages(h,"snapshot","",0,&error);assert(batch && !error && horus_mesh_batch_size(batch)==1);
    horus_mesh_batch_destroy(batch);
    horus_mesh_source_destroy(h);
}
