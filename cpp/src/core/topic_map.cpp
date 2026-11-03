#include "horus/core/topic_map.hpp"

#include "horus/core/event_bus.hpp"
#include <algorithm>
#include <cctype>
#include <chrono>
#include <sstream>
#include <stdexcept>

namespace horus {
namespace core {

namespace {
double now() {
    return std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch())
        .count();
}
std::string strip(const std::string &value) {
    auto a = value.find_first_not_of('/'), b = value.find_last_not_of('/');
    return a == std::string::npos ? "" : value.substr(a, b - a + 1);
}
std::string detected_robot(const std::string &name) {
    auto first = strip(name);
    first = first.substr(0, first.find('/'));
    auto lower = first;
    std::transform(lower.begin(), lower.end(), lower.begin(),
                   [](unsigned char c) { return std::tolower(c); });
    for (auto word : {"robot", "carter", "tb3", "turtlebot", "husky", "jackal"})
        if (lower.find(word) != std::string::npos)
            return first;
    return {};
}
} // namespace
bool TopicInfo::is_sensor_topic() const {
    return topic_type == TopicType::IMAGE || topic_type == TopicType::POINT_CLOUD ||
           topic_type == TopicType::LASER_SCAN || topic_type == TopicType::IMU ||
           topic_type == TopicType::GPS;
}
bool TopicInfo::is_robot_topic() const { return !robot_namespace.empty(); }
std::string TopicInfo::get_horus_event_topic() const {
    if (!robot_namespace.empty()) {
        if (!sensor_name.empty())
            return robot_namespace + ".sensor." + sensor_name;
        const auto suffix = topic_type == TopicType::ROBOT_STATUS ? "status"
                            : topic_type == TopicType::ODOMETRY   ? "odometry"
                            : topic_type == TopicType::PATH       ? "path"
                                                                  : "data";
        return robot_namespace + "." + suffix;
    }
    return (topic_type == TopicType::TF || topic_type == TopicType::TF_STATIC) ? "global.tf"
           : topic_type == TopicType::OCCUPANCY_GRID                           ? "global.map"
                                                                               : "global.data";
}
TopicMap::TopicMap(const TopicMap &other) {
    std::lock_guard lock(other.mutex_);
    topic_mappings_ = other.topic_mappings_;
}
TopicMap &TopicMap::operator=(const TopicMap &other) {
    if (this != &other) {
        std::scoped_lock lock(mutex_, other.mutex_);
        topic_mappings_ = other.topic_mappings_;
    }
    return *this;
}

TopicInfo TopicMap::prepare_topic(const std::string &topic_name, TopicInfo topic) {
    topic.topic_name = topic_name;
    if (topic.namespace_prefix.empty()) {
        topic.namespace_prefix = infer_namespace(topic_name);
    }
    if (topic.sensor_name.empty()) {
        topic.sensor_name = infer_sensor(topic_name);
    }
    if (topic.robot_namespace.empty())
        topic.robot_namespace = detected_robot(topic_name);
    if (topic.last_seen == 0)
        topic.last_seen = now();
    return topic;
}

bool TopicMap::add_topic(const std::string &topic_name, const TopicInfo &topic_info) {
    if (topic_name.empty())
        return false;
    auto topic = prepare_topic(topic_name, topic_info);
    {
        std::lock_guard lock(mutex_);
        topic_mappings_[topic_name] = topic;
    }
    publish("topicmap.topic.discovered",
            Metadata{{"topic_name", topic_name},
                     {"topic_type", topic_type_to_string(topic.topic_type)},
                     {"robot_namespace", topic.robot_namespace},
                     {"sensor_name", topic.sensor_name}},
            EventPriority::NORMAL, "TopicMap");
    return true;
}

TopicInfo TopicMap::add_topic(const std::string &name, TopicType type, TopicDirection direction,
                              TopicInfo options) {
    options.topic_type = type;
    options.direction = direction;
    options = prepare_topic(name, std::move(options));
    if (!add_topic(name, options))
        throw std::invalid_argument("Topic name cannot be empty");
    return options;
}
TopicInfo TopicMap::add_topic(const std::string &name, const std::string &ros_type,
                              TopicDirection direction, TopicInfo options) {
    auto normalized = ros_type;
    if (auto at = normalized.find("/msg/"); at != std::string::npos)
        normalized.replace(at, 5, "/");
    TopicType type = TopicType::UNKNOWN;
    for (int value = int(TopicType::IMAGE); value <= int(TopicType::UNKNOWN); ++value)
        if (topic_type_to_string(static_cast<TopicType>(value)) == normalized) {
            type = static_cast<TopicType>(value);
            break;
        }
    return add_topic(name, type, direction, std::move(options));
}

bool TopicMap::remove_topic(const std::string &topic_name) {
    std::optional<TopicInfo> removed;
    {
        std::lock_guard lock(mutex_);
        auto it = topic_mappings_.find(topic_name);
        if (it == topic_mappings_.end())
            return false;
        removed = std::move(it->second);
        topic_mappings_.erase(it);
    }
    publish("topicmap.topic.removed",
            Metadata{{"topic_name", topic_name}, {"robot_namespace", removed->robot_namespace}},
            EventPriority::NORMAL, "TopicMap");
    return true;
}

std::optional<TopicInfo> TopicMap::get_topic_info(const std::string &topic_name) const {
    std::lock_guard lock(mutex_);
    const auto it = topic_mappings_.find(topic_name);
    if (it == topic_mappings_.end()) {
        return std::nullopt;
    }
    return it->second;
}

std::vector<std::string> TopicMap::get_topics_by_type(TopicType type) const {
    std::lock_guard lock(mutex_);
    std::vector<std::string> result;
    for (const auto &[name, info] : topic_mappings_) {
        if (info.topic_type == type) {
            result.push_back(name);
        }
    }
    return result;
}

std::vector<std::string>
TopicMap::get_topics_by_namespace(const std::string &namespace_prefix) const {
    std::lock_guard lock(mutex_);
    std::vector<std::string> result;
    for (const auto &[name, info] : topic_mappings_) {
        if (info.namespace_prefix == namespace_prefix) {
            result.push_back(name);
        }
    }
    return result;
}

std::vector<std::string> TopicMap::get_topics_by_sensor(const std::string &sensor_name) const {
    std::lock_guard lock(mutex_);
    std::vector<std::string> result;
    for (const auto &[name, info] : topic_mappings_) {
        if (info.sensor_name == sensor_name) {
            result.push_back(name);
        }
    }
    return result;
}

std::map<std::string, std::size_t> TopicMap::get_topic_stats() const {
    std::lock_guard lock(mutex_);
    std::size_t active = 0;
    for (const auto &[_, info] : topic_mappings_) {
        if (info.active) {
            active++;
        }
    }
    return {
        {"total_topics", topic_mappings_.size()},
        {"active_topics", active},
        {"inactive_topics", topic_mappings_.size() - active},
    };
}

std::unordered_map<std::string, TopicInfo> TopicMap::get_topic_mappings() const {
    std::lock_guard lock(mutex_);
    return topic_mappings_;
}

std::vector<std::string> TopicMap::get_all_topics() const {
    std::lock_guard lock(mutex_);
    std::vector<std::string> topics;
    topics.reserve(topic_mappings_.size());
    for (const auto &[topic, _] : topic_mappings_) {
        topics.push_back(topic);
    }
    std::sort(topics.begin(), topics.end());
    return topics;
}

std::size_t TopicMap::size() const {
    std::lock_guard lock(mutex_);
    return topic_mappings_.size();
}

std::string TopicMap::infer_namespace(const std::string &topic_name) {
    if (topic_name.empty() || topic_name == "/") {
        return "/";
    }
    std::stringstream stream(topic_name);
    std::string segment;
    std::vector<std::string> parts;
    while (std::getline(stream, segment, '/')) {
        if (!segment.empty()) {
            parts.push_back(segment);
        }
    }
    if (parts.size() <= 1) {
        return "/";
    }
    return "/" + parts[0];
}

std::string TopicMap::infer_sensor(const std::string &topic_name) {
    std::stringstream stream(topic_name);
    std::string segment;
    while (std::getline(stream, segment, '/')) {
        auto lower = segment;
        std::transform(lower.begin(), lower.end(), lower.begin(),
                       [](unsigned char c) { return std::tolower(c); });
        for (auto word : {"camera", "lidar", "laser", "scan", "imu", "gps", "depth"})
            if (lower.find(word) != std::string::npos)
                return segment;
    }
    return {};
}

std::vector<TopicInfo> TopicMap::get_topics_by_robot(const std::string &name) const {
    std::lock_guard lock(mutex_);
    std::vector<TopicInfo> out;
    for (const auto &[_, info] : topic_mappings_)
        if (info.robot_namespace == name)
            out.push_back(info);
    return out;
}
std::vector<TopicInfo> TopicMap::get_sensor_topics(const std::string &name) const {
    std::lock_guard lock(mutex_);
    std::vector<TopicInfo> out;
    for (const auto &[_, info] : topic_mappings_)
        if (info.is_sensor_topic() && (name.empty() || info.robot_namespace == name))
            out.push_back(info);
    return out;
}
std::set<std::string> TopicMap::get_robot_namespaces() const {
    std::lock_guard lock(mutex_);
    std::set<std::string> out;
    for (const auto &[_, info] : topic_mappings_)
        if (info.is_robot_topic())
            out.insert(info.robot_namespace);
    return out;
}
void TopicMap::update_topic_activity(const std::string &name, bool active) {
    {
        std::lock_guard lock(mutex_);
        auto it = topic_mappings_.find(name);
        if (it == topic_mappings_.end())
            return;
        it->second.active = active;
        it->second.last_seen = now();
    }
    publish("topicmap.topic.activity", Metadata{{"topic_name", name}, {"active", active}},
            EventPriority::LOW, "TopicMap");
}
std::map<std::string, std::string> TopicMap::create_topic_mapping(const std::string &name) const {
    std::map<std::string, std::string> out;
    for (const auto &info : get_sensor_topics(name))
        if (!info.sensor_name.empty())
            out[info.sensor_name] = info.topic_name;
    return out;
}
nlohmann::json TopicMap::get_stats() const {
    std::lock_guard lock(mutex_);
    std::set<std::string> robots;
    size_t active = 0, sensors = 0;
    nlohmann::json counts = nlohmann::json::object();
    for (int i = int(TopicType::IMAGE); i <= int(TopicType::UNKNOWN); ++i)
        counts[topic_type_to_string(static_cast<TopicType>(i))] = 0;
    for (const auto &[_, info] : topic_mappings_) {
        active += info.active;
        sensors += info.is_sensor_topic();
        if (info.is_robot_topic())
            robots.insert(info.robot_namespace);
        auto &count = counts[topic_type_to_string(info.topic_type)];
        count = count.get<size_t>() + 1;
    }
    return {{"total_topics", topic_mappings_.size()},
            {"active_topics", active},
            {"robot_count", robots.size()},
            {"sensor_topics", sensors},
            {"topic_types", counts}};
}

TopicMap &get_topic_map() {
    static TopicMap map;
    return map;
}

} // namespace core
} // namespace horus
