#ifndef HORUS_CORE_EVENT_BUS_HPP
#define HORUS_CORE_EVENT_BUS_HPP

#include "horus/core/types.hpp"
#include <string>
#include <memory>
#include <functional>
#include <vector>
#include <queue>
#include <map>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <chrono>
#include <atomic>
#include <any>

namespace horus {
namespace core {

/**
 * @brief Event class for EventBus communication
 */
struct Event {
    std::string topic;
    std::any data;
    EventPriority priority;
    std::string source;
    std::string event_id;
    double timestamp;
    
    Event(const std::string& topic, 
          const std::any& data = std::any(), 
          EventPriority priority = EventPriority::NORMAL,
          const std::string& source = "");
};

/**
 * @brief Event subscription callback type
 */
using EventCallback = std::function<void(const Event&)>;

/**
 * @brief Manages a single event subscription
 */
class EventSubscription {
public:
    EventSubscription(EventCallback callback,
                     const std::string& topic_filter,
                     EventPriority priority_filter = EventPriority::LOW);
    
    bool matches(const Event& event) const;
    void invoke(const Event& event) const;
    
    std::string get_id() const { return subscription_id_; }
    bool is_active() const { return active_; }
    void deactivate() { active_ = false; }
    
private:
    bool matches_topic(const std::string& event_topic, 
                      const std::string& topic_filter) const;
    
    EventCallback callback_;
    std::string topic_filter_;
    EventPriority priority_filter_;
    std::string subscription_id_;
    bool active_;
};

/**
 * @brief Thread-safe event bus for HORUS SDK component communication
 * 
 * Enables real-time data flow between ROS robots and Meta Quest 3 application
 * through topic-based publish/subscribe messaging with priority handling.
 */
class EventBus {
public:
    EventBus(size_t max_queue_size = 1000);
    ~EventBus();
    
    // Prevent copying
    EventBus(const EventBus&) = delete;
    EventBus& operator=(const EventBus&) = delete;
    
    /**
     * @brief Start the event processing system
     */
    void start();
    
    /**
     * @brief Stop the event processing system
     */
    void stop();
    
    /**
     * @brief Publish an event to the bus
     * @param topic Event topic string (e.g., "robot1.sensor.camera")
     * @param data Event data
     * @param priority Event priority level
     * @param source Optional event source identifier
     */
    void publish(const std::string& topic,
                const std::any& data = std::any(),
                EventPriority priority = EventPriority::NORMAL,
                const std::string& source = "");
    
    /**
     * @brief Publish an event object
     */
    void publish(const Event& event);
    
    /**
     * @brief Subscribe to events matching topic filter
     * @param topic_filter Topic pattern to match (supports * and + wildcards)
     * @param callback Function to call when matching event is received
     * @param priority_filter Optional minimum priority level
     * @return Subscription ID for later unsubscription
     */
    std::string subscribe(const std::string& topic_filter,
                         EventCallback callback,
                         EventPriority priority_filter = EventPriority::LOW);
    
    /**
     * @brief Unsubscribe from events
     * @param subscription_id ID returned from subscribe()
     * @return True if subscription was found and removed
     */
    bool unsubscribe(const std::string& subscription_id);
    
    /**
     * @brief Unsubscribe all subscriptions, optionally filtered by topic
     * @param topic_filter If provided, only unsubscribe from this topic
     */
    void unsubscribe_all(const std::string& topic_filter = "");
    
    /**
     * @brief Get event bus statistics
     */
    std::map<std::string, size_t> get_stats() const;
    
    /**
     * @brief Get current subscription counts by topic
     */
    std::map<std::string, size_t> get_subscriptions() const;
    
private:
    void process_events();
    void dispatch_event(const Event& event);
    
    struct EventComparator {
        bool operator()(const Event& a, const Event& b) const {
            return static_cast<int>(a.priority) < static_cast<int>(b.priority);
        }
    };
    
    mutable std::recursive_mutex mutex_;
    std::condition_variable_any queue_condition_;
    std::priority_queue<Event, std::vector<Event>, EventComparator> event_queue_;
    std::map<std::string, std::vector<std::shared_ptr<EventSubscription>>> subscriptions_;
    
    std::atomic<bool> running_;
    std::unique_ptr<std::thread> processing_thread_;
    size_t max_queue_size_;
    
    // Statistics
    mutable std::atomic<size_t> events_published_;
    mutable std::atomic<size_t> events_processed_;
    mutable std::atomic<size_t> dropped_events_;
};

/**
 * @brief Get the global HORUS event bus instance
 */
EventBus& get_event_bus();

/**
 * @brief Convenience functions for global event bus
 */
void publish(const std::string& topic,
            const std::any& data = std::any(),
            EventPriority priority = EventPriority::NORMAL,
            const std::string& source = "");

std::string subscribe(const std::string& topic_filter,
                     EventCallback callback,
                     EventPriority priority_filter = EventPriority::LOW);

bool unsubscribe(const std::string& subscription_id);

} // namespace core
} // namespace horus

#endif // HORUS_CORE_EVENT_BUS_HPP