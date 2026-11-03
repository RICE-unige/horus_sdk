#include "horus/core/event_bus.hpp"
#include <algorithm>
#include <ctime>
#include <deque>
#include <iomanip>
#include <iostream>
#include <optional>
#include <random>
#include <sstream>

namespace horus {
namespace core {

// Generate unique ID
static std::string generate_uuid() {
    thread_local std::mt19937 gen(std::random_device{}());
    thread_local std::uniform_int_distribution<> dis(0, 15);

    std::stringstream ss;
    ss << std::hex;
    for (int i = 0; i < 32; ++i) {
        ss << dis(gen);
        if (i == 7 || i == 11 || i == 15 || i == 19)
            ss << "-";
    }
    return ss.str();
}

static double get_timestamp() {
    auto now = std::chrono::system_clock::now();
    auto duration = now.time_since_epoch();
    return std::chrono::duration_cast<std::chrono::milliseconds>(duration).count() / 1000.0;
}

Event::Event(const std::string &topic, const std::any &data, EventPriority priority,
             const std::string &source)
    : topic(topic), data(data), priority(priority), source(source), event_id(generate_uuid()),
      timestamp(get_timestamp()) {}

EventSubscription::EventSubscription(EventCallback callback, const std::string &topic_filter,
                                     EventPriority priority_filter)
    : callback_(callback), topic_filter_(topic_filter), priority_filter_(priority_filter),
      subscription_id_(generate_uuid()), active_(true) {}

EventSubscription::EventSubscription(const EventSubscription &other)
    : callback_(other.callback_), topic_filter_(other.topic_filter_),
      priority_filter_(other.priority_filter_), subscription_id_(other.subscription_id_),
      active_(other.active_.load()) {}

EventSubscription &EventSubscription::operator=(const EventSubscription &other) {
    if (this != &other) {
        callback_ = other.callback_;
        topic_filter_ = other.topic_filter_;
        priority_filter_ = other.priority_filter_;
        subscription_id_ = other.subscription_id_;
        active_.store(other.active_.load());
    }
    return *this;
}

bool EventSubscription::matches(const Event &event) const {
    if (!active_)
        return false;
    if (!matches_topic(event.topic, topic_filter_))
        return false;
    if (static_cast<int>(event.priority) < static_cast<int>(priority_filter_))
        return false;
    return true;
}

void EventSubscription::invoke(const Event &event) const {
    if (callback_)
        callback_(event);
}

bool EventSubscription::matches_topic(const std::string &event_topic,
                                      const std::string &topic_filter) const {
    if (topic_filter == "*")
        return true;
    if (event_topic == topic_filter)
        return true;

    if (!topic_filter.empty() && topic_filter.back() == '*') {
        std::string prefix = topic_filter.substr(0, topic_filter.size() - 1);
        return event_topic.find(prefix) == 0;
    }

    auto split = [](const std::string &value) {
        std::vector<std::string> parts;
        std::stringstream stream(value);
        std::string token;
        while (std::getline(stream, token, '.')) {
            parts.push_back(token);
        }
        return parts;
    };

    const auto filter_parts = split(topic_filter);
    const auto event_parts = split(event_topic);
    if (filter_parts.empty()) {
        return false;
    }

    if (!filter_parts.empty() && filter_parts.back() == "*") {
        if (event_parts.size() < filter_parts.size() - 1) {
            return false;
        }
        for (std::size_t i = 0; i + 1 < filter_parts.size(); ++i) {
            if (filter_parts[i] != "+" && filter_parts[i] != event_parts[i]) {
                return false;
            }
        }
        return true;
    }

    if (filter_parts.size() != event_parts.size()) {
        return false;
    }
    for (std::size_t i = 0; i < filter_parts.size(); ++i) {
        if (filter_parts[i] != "+" && filter_parts[i] != event_parts[i]) {
            return false;
        }
    }
    return true;
}

struct EventBus::State {
    std::mutex mutex;
    std::condition_variable ready;
    std::deque<Event> queue;
    std::map<std::string, std::vector<std::shared_ptr<EventSubscription>>> subscriptions;
    bool running{};
    std::uint64_t generation{};
    size_t maximum, published{}, processed{}, dropped{};
    explicit State(size_t maximum) : maximum(std::max(size_t(1), maximum)) {}
};
EventBus::EventBus(size_t max_queue_size) : state_(std::make_shared<State>(max_queue_size)) {}

EventBus::~EventBus() { stop(); }

void EventBus::start() {
    std::lock_guard lifecycle(lifecycle_mutex_);
    std::lock_guard lock(state_->mutex);
    if (state_->running)
        return;
    // A callback may stop/restart its own bus. Old workers retain only shared state.
    if (processing_thread_.joinable())
        processing_thread_.detach();
    state_->running = true;
    const auto generation = ++state_->generation;
    try {
        processing_thread_ = std::thread(&EventBus::process_events, state_, generation);
    } catch (...) {
        state_->running = false;
        throw;
    }
}

void EventBus::stop() {
    std::thread worker;
    {
        std::lock_guard lifecycle(lifecycle_mutex_);
        std::lock_guard lock(state_->mutex);
        state_->running = false;
        ++state_->generation;
        state_->ready.notify_all();
        worker = std::move(processing_thread_);
    }
    if (worker.joinable()) {
        if (worker.get_id() == std::this_thread::get_id())
            worker.detach();
        else
            worker.join();
    }
}

void EventBus::publish(const std::string &topic, const std::any &data, EventPriority priority,
                       const std::string &source) {
    publish(Event(topic, data, priority, source));
}

void EventBus::publish(const Event &event) {
    std::lock_guard lock(state_->mutex);
    if (state_->queue.size() >= state_->maximum) {
        auto victim = std::find_if(state_->queue.begin(), state_->queue.end(),
                                   [](const Event &e) { return e.priority == EventPriority::LOW; });
        state_->queue.erase(victim == state_->queue.end() ? state_->queue.begin() : victim);
        ++state_->dropped;
    }
    state_->queue.push_back(event);
    ++state_->published;
    state_->ready.notify_one();
}

std::string EventBus::subscribe(const std::string &topic_filter, EventCallback callback,
                                EventPriority priority_filter) {
    auto subscription =
        std::make_shared<EventSubscription>(callback, topic_filter, priority_filter);
    std::lock_guard lock(state_->mutex);
    state_->subscriptions[topic_filter].push_back(subscription);
    return subscription->get_id();
}

bool EventBus::unsubscribe(const std::string &subscription_id) {
    std::lock_guard lock(state_->mutex);
    for (auto &[topic, subs] : state_->subscriptions) {
        auto it = std::find_if(subs.begin(), subs.end(), [&subscription_id](const auto &sub) {
            return sub->get_id() == subscription_id;
        });
        if (it != subs.end()) {
            subs.erase(it);
            if (subs.empty())
                state_->subscriptions.erase(topic);
            return true;
        }
    }
    return false;
}

void EventBus::unsubscribe_all(const std::string &topic_filter) {
    std::lock_guard lock(state_->mutex);
    if (topic_filter.empty()) {
        state_->subscriptions.clear();
    } else {
        state_->subscriptions.erase(topic_filter);
    }
}

std::map<std::string, size_t> EventBus::get_stats() const {
    std::lock_guard lock(state_->mutex);
    size_t count = 0;
    for (const auto &[_, sub] : state_->subscriptions)
        count += sub.size();
    return {{"events_published", state_->published},
            {"events_processed", state_->processed},
            {"dropped_events", state_->dropped},
            {"queue_size", state_->queue.size()},
            {"active_subscriptions", count}};
}

std::map<std::string, size_t> EventBus::get_subscriptions() const {
    std::lock_guard lock(state_->mutex);
    std::map<std::string, size_t> result;
    for (const auto &[topic, subs] : state_->subscriptions) {
        result[topic] = subs.size();
    }
    return result;
}

void EventBus::process_events(std::shared_ptr<State> state, std::uint64_t generation) {
    while (true) {
        std::optional<Event> event;
        {
            std::unique_lock lock(state->mutex);
            state->ready.wait(
                lock, [&] { return !state->queue.empty() || state->generation != generation; });
            if (state->generation != generation)
                return;
            auto best = std::max_element(
                state->queue.begin(), state->queue.end(),
                [](const Event &a, const Event &b) { return a.priority < b.priority; });
            event = std::move(*best);
            state->queue.erase(best);
        }
        dispatch_event(state, *event);
        std::lock_guard lock(state->mutex);
        ++state->processed;
    }
}

void EventBus::dispatch_event(const std::shared_ptr<State> &state, const Event &event) {
    std::vector<std::shared_ptr<EventSubscription>> matching_subs;
    {
        std::lock_guard lock(state->mutex);
        for (const auto &[topic, subs] : state->subscriptions) {
            for (const auto &sub : subs) {
                if (sub->matches(event)) {
                    matching_subs.push_back(sub);
                }
            }
        }
    }
    for (const auto &sub : matching_subs) {
        try {
            sub->invoke(event);
        } catch (const std::exception &e) {
            std::cerr << "HORUS event callback failed: " << e.what() << '\n';
        } catch (...) {
            std::cerr << "HORUS event callback failed\n";
        }
    }
}

static std::unique_ptr<EventBus> global_event_bus;
static std::mutex global_mutex;

EventBus &get_event_bus() {
    std::lock_guard<std::mutex> lock(global_mutex);
    if (!global_event_bus) {
        global_event_bus = std::make_unique<EventBus>();
        global_event_bus->start();
    }
    return *global_event_bus;
}

void publish(const std::string &topic, const std::any &data, EventPriority priority,
             const std::string &source) {
    get_event_bus().publish(topic, data, priority, source);
}

std::string subscribe(const std::string &topic_filter, EventCallback callback,
                      EventPriority priority_filter) {
    return get_event_bus().subscribe(topic_filter, callback, priority_filter);
}

bool unsubscribe(const std::string &subscription_id) {
    return get_event_bus().unsubscribe(subscription_id);
}

} // namespace core
} // namespace horus
