#include "horus/mapping/mesh_page_scheduler.hpp"
#include <nlohmann/json.hpp>
#include <fstream>
#include <limits>
#include <stdexcept>
using namespace horus::mapping;
using Json = nlohmann::json;
void check(bool result) { if (!result) throw std::runtime_error("Scheduler contract failed"); }
MeshPage page(const Json& a) {
    return std::make_shared<const MeshPageUpdate>(MeshPageUpdate{a[0].get<std::uint32_t>(),
        a[1].get<std::uint64_t>(), a[2].get<std::uint64_t>(),
        static_cast<MeshOperation>(a[3].get<int>()), std::vector<std::uint8_t>(a[4].get<std::size_t>()), 3, 1});
}
int main() {
    std::ifstream input(HORUS_SCHEDULER_FIXTURE);
    Json cases; input >> cases;
    const MeshSchedulerOptions options{10., 1000., 4, 2000};
    for (const auto& test : cases) {
        MeshPageScheduler queue(options);
        for (const auto& event : test["events"]) {
            if (event.contains("enqueue")) check(queue.enqueue(page(event["enqueue"])) == event["accepted"]);
            else {
                const auto ready = queue.pop_ready(event["pop"]);
                if (event["ready"].is_null()) check(!ready);
                else {
                    check(bool(ready));
                    check(Json::array({ready->chunk_id, ready->epoch, ready->revision,
                        static_cast<int>(ready->operation), ready->payload.size()}) == event["ready"]);
                }
            }
            if (event.contains("count")) check(queue.pending_count() == event["count"]);
            if (event.contains("bytes")) check(queue.pending_bytes() == event["bytes"]);
        }
    }
    MeshPageScheduler queue(options);
    auto shared = page({1,1,1,0,100});
    queue.enqueue(shared);
    check(queue.pop_ready(1).get() == shared.get());
    for (int id = 1; id <= 4; ++id) queue.enqueue(page({id,1,2,0,100}));
    for (const auto& invalid : {page({5,1,2,0,100}), page({1,9,2,0,2001})}) {
        bool failed = false;
        try { queue.enqueue(invalid); } catch (const std::exception&) { failed = true; }
        check(failed && queue.pending_count() == 4 && queue.pending_bytes() == 400);
    }
    for (double value : {std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN()}) {
        bool failed = false;
        try { queue.pop_ready(value); } catch (const std::invalid_argument&) { failed = true; }
        check(failed && queue.pending_count() == 4);
        failed = false;
        try { MeshPageScheduler bad(options, value); } catch (const std::invalid_argument&) { failed = true; }
        check(failed);
    }
}
