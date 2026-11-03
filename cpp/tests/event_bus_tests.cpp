#include "horus/core/event_bus.hpp"
#include <future>
#include <set>
#include <stdexcept>

using namespace horus::core;
using namespace std::chrono_literals;
void check(bool condition) {
    if (!condition)
        throw std::runtime_error("Event bus contract failed");
}
int main() {
    EventBus bus(3);
    std::mutex mutex;
    std::condition_variable ready;
    std::vector<int> seen;
    bus.subscribe("robot.+", [&](const Event &e) {
        std::lock_guard lock(mutex);
        seen.push_back(std::any_cast<int>(e.data));
        ready.notify_all();
    });
    auto second = bus.subscribe(
        "robot.+", [](const Event &) { throw std::runtime_error("expected callback failure"); });
    check(bus.get_stats().at("active_subscriptions") == 2);
    bus.publish("robot.status", 0, EventPriority::LOW);
    bus.publish("robot.status", 1, EventPriority::NORMAL);
    bus.publish("robot.status", 2, EventPriority::HIGH);
    bus.publish("robot.status", 3, EventPriority::HIGH);
    check(bus.get_stats().at("dropped_events") == 1);
    bus.start();
    {
        std::unique_lock lock(mutex);
        check(ready.wait_for(lock, 2s, [&] { return seen.size() == 3; }));
        check(seen == std::vector<int>({2, 3, 1}));
    }
    bus.stop();
    check(bus.unsubscribe(second));
    auto stopped = std::make_shared<std::promise<void>>();
    auto future = stopped->get_future();
    bus.subscribe("stop", [&](const Event &) {
        bus.stop();
        stopped->set_value();
    });
    bus.publish("stop", {}, EventPriority::CRITICAL);
    bus.start();
    check(future.wait_for(2s) == std::future_status::ready);
    bus.publish("robot.status", 4);
    bus.start();
    {
        std::unique_lock lock(mutex);
        check(ready.wait_for(lock, 2s, [&] { return seen.size() == 4; }));
    }
    bus.stop();
    bus.unsubscribe_all();
    check(bus.get_stats().at("active_subscriptions") == 0);
    EventBus concurrent(4096);
    std::set<std::string> identifiers;
    concurrent.subscribe("*", [&](const Event &e) {
        std::lock_guard lock(mutex);
        identifiers.insert(e.event_id);
        ready.notify_all();
    });
    concurrent.start();
    std::vector<std::thread> producers;
    for (int i = 0; i < 4; ++i)
        producers.emplace_back([&] {
            for (int k = 0; k < 500; ++k)
                concurrent.publish("event");
        });
    for (auto &producer : producers)
        producer.join();
    {
        std::unique_lock lock(mutex);
        check(ready.wait_for(lock, 5s, [&] { return identifiers.size() == 2000; }));
    }
    concurrent.stop();
    check(concurrent.get_stats().at("events_processed") == 2000);
    // Self-destruction from a callback cannot leave a worker referring to the bus object.
    auto owned = std::make_unique<EventBus>();
    std::promise<void> destroyed;
    auto done = destroyed.get_future();
    owned->subscribe("destroy", [&](const Event &) {
        owned.reset();
        destroyed.set_value();
    });
    owned->publish("destroy");
    owned->start();
    check(done.wait_for(2s) == std::future_status::ready);
}
