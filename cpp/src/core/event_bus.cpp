#include "horus/core/event_bus.hpp"
#include <algorithm>
#include <sstream>
#include <iomanip>
#include <ctime>
#include <random>

namespace horus {
namespace core {

// Generate unique ID
static std::string generate_uuid() {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    static std::uniform_int_distribution<> dis(0, 15);
    
    std::stringstream ss;
    ss << std::hex;
    for (int i = 0; i < 32; ++i) {
        ss << dis(gen);
        if (i == 7 || i == 11 || i == 15 || i == 19) ss << "-";
    }
    return ss.str();
}

static double get_timestamp() {
    auto now = std::chrono::system_clock::now();
    auto duration = now.time_since_epoch();
    return std::chrono::duration_cast<std::chrono::milliseconds>(duration).count() / 1000.0;
}

Event::Event(const std::string& topic, const std::any& data, 
             EventPriority priority, const std::string& source)
    : topic(topic), data(data), priority(priority), source(source)
    , event_id(generate_uuid()), timestamp(get_timestamp()) {}

EventSubscription::EventSubscription(EventCallback callback,
                                   const std::string& topic_filter,
                                   EventPriority priority_filter)
    : callback_(callback), topic_filter_(topic_filter)
    , priority_filter_(priority_filter)
    , subscription_id_(generate_uuid()), active_(true) {}

bool EventSubscription::matches(const Event& event) const {
    if (!active_) return false;
    if (!matches_topic(event.topic, topic_filter_)) return false;
    if (static_cast<int>(event.priority) < static_cast<int>(priority_filter_)) 
        return false;
    return true;
}

void EventSubscription::invoke(const Event& event) const {
    if (callback_) callback_(event);
}

bool EventSubscription::matches_topic(const std::string& event_topic, 
                                      const std::string& topic_filter) const {
    if (topic_filter == "*") return true;
    if (event_topic == topic_filter) return true;
    
    if (!topic_filter.empty() && topic_filter.back() == '*') {
        std::string prefix = topic_filter.substr(0, topic_filter.size() - 1);
        return event_topic.find(prefix) == 0;
    }

    auto split = [](const std::string& value) {
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

EventBus::EventBus(size_t max_queue_size)
    : running_(false), max_queue_size_(max_queue_size)
    , events_published_(0), events_processed_(0), dropped_events_(0) {}

EventBus::~EventBus() { stop(); }

void EventBus::start() {
    if (running_.exchange(true)) return;
    processing_thread_ = std::make_unique<std::thread>(&EventBus::process_events, this);
}

void EventBus::stop() {
    if (!running_.exchange(false)) return;
    queue_condition_.notify_all();
    if (processing_thread_ && processing_thread_->joinable()) {
        processing_thread_->join();
    }
}

void EventBus::publish(const std::string& topic, const std::any& data,
                      EventPriority priority, const std::string& source) {
    publish(Event(topic, data, priority, source));
}

void EventBus::publish(const Event& event) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (event_queue_.size() >= max_queue_size_) {
        dropped_events_++;
        return;
    }
    event_queue_.push(event);
    events_published_++;
    queue_condition_.notify_one();
}

std::string EventBus::subscribe(const std::string& topic_filter,
                                EventCallback callback,
                                EventPriority priority_filter) {
    auto subscription = std::make_shared<EventSubscription>(
        callback, topic_filter, priority_filter);
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    subscriptions_[topic_filter].push_back(subscription);
    return subscription->get_id();
}

bool EventBus::unsubscribe(const std::string& subscription_id) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    for (auto& [topic, subs] : subscriptions_) {
        auto it = std::find_if(subs.begin(), subs.end(),
            [&subscription_id](const auto& sub) {
                return sub->get_id() == subscription_id;
            });
        if (it != subs.end()) {
            subs.erase(it);
            if (subs.empty()) subscriptions_.erase(topic);
            return true;
        }
    }
    return false;
}

void EventBus::unsubscribe_all(const std::string& topic_filter) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (topic_filter.empty()) {
        subscriptions_.clear();
    } else {
        subscriptions_.erase(topic_filter);
    }
}

std::map<std::string, size_t> EventBus::get_stats() const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return {
        {"events_published", events_published_.load()},
        {"events_processed", events_processed_.load()},
        {"dropped_events", dropped_events_.load()},
        {"queue_size", event_queue_.size()},
        {"active_subscriptions", subscriptions_.size()}
    };
}

std::map<std::string, size_t> EventBus::get_subscriptions() const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    std::map<std::string, size_t> result;
    for (const auto& [topic, subs] : subscriptions_) {
        result[topic] = subs.size();
    }
    return result;
}

void EventBus::process_events() {
    while (running_) {
        Event event("", std::any(), EventPriority::NORMAL, "");
        bool has_event = false;
        {
            std::unique_lock<std::recursive_mutex> lock(mutex_);
            queue_condition_.wait(lock, [this] {
                return !event_queue_.empty() || !running_;
            });
            if (!running_) break;
            if (!event_queue_.empty()) {
                event = event_queue_.top();
                event_queue_.pop();
                has_event = true;
            }
        }
        if (has_event) {
            dispatch_event(event);
            events_processed_++;
        }
    }
}

void EventBus::dispatch_event(const Event& event) {
    std::vector<std::shared_ptr<EventSubscription>> matching_subs;
    {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        for (const auto& [topic, subs] : subscriptions_) {
            for (const auto& sub : subs) {
                if (sub->matches(event)) {
                    matching_subs.push_back(sub);
                }
            }
        }
    }
    for (const auto& sub : matching_subs) {
        try { sub->invoke(event); } 
        catch (const std::exception& e) {}
    }
}

static std::unique_ptr<EventBus> global_event_bus;
static std::mutex global_mutex;

EventBus& get_event_bus() {
    std::lock_guard<std::mutex> lock(global_mutex);
    if (!global_event_bus) {
        global_event_bus = std::make_unique<EventBus>();
        global_event_bus->start();
    }
    return *global_event_bus;
}

void publish(const std::string& topic, const std::any& data,
            EventPriority priority, const std::string& source) {
    get_event_bus().publish(topic, data, priority, source);
}

std::string subscribe(const std::string& topic_filter,
                     EventCallback callback,
                     EventPriority priority_filter) {
    return get_event_bus().subscribe(topic_filter, callback, priority_filter);
}

bool unsubscribe(const std::string& subscription_id) {
    return get_event_bus().unsubscribe(subscription_id);
}

} // namespace core
} // namespace horus
