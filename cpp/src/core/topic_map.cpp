#include "horus/core/topic_map.hpp"

#include <algorithm>
#include <sstream>

namespace horus {
namespace core {

bool TopicMap::add_topic(const std::string& topic_name, const TopicInfo& topic_info) {
    if (topic_name.empty()) {
        return false;
    }
    auto topic = topic_info;
    topic.topic_name = topic_name;
    if (topic.namespace_prefix.empty()) {
        topic.namespace_prefix = infer_namespace(topic_name);
    }
    if (topic.sensor_name.empty()) {
        topic.sensor_name = infer_sensor(topic_name);
    }
    topic_mappings_[topic_name] = topic;
    return true;
}

bool TopicMap::remove_topic(const std::string& topic_name) {
    return topic_mappings_.erase(topic_name) > 0;
}

std::optional<TopicInfo> TopicMap::get_topic_info(const std::string& topic_name) const {
    const auto it = topic_mappings_.find(topic_name);
    if (it == topic_mappings_.end()) {
        return std::nullopt;
    }
    return it->second;
}

std::vector<std::string> TopicMap::get_topics_by_type(TopicType type) const {
    std::vector<std::string> result;
    for (const auto& [name, info] : topic_mappings_) {
        if (info.topic_type == type) {
            result.push_back(name);
        }
    }
    return result;
}

std::vector<std::string> TopicMap::get_topics_by_namespace(const std::string& namespace_prefix) const {
    std::vector<std::string> result;
    for (const auto& [name, info] : topic_mappings_) {
        if (info.namespace_prefix == namespace_prefix) {
            result.push_back(name);
        }
    }
    return result;
}

std::vector<std::string> TopicMap::get_topics_by_sensor(const std::string& sensor_name) const {
    std::vector<std::string> result;
    for (const auto& [name, info] : topic_mappings_) {
        if (info.sensor_name == sensor_name) {
            result.push_back(name);
        }
    }
    return result;
}

std::map<std::string, std::size_t> TopicMap::get_topic_stats() const {
    std::size_t active = 0;
    for (const auto& [_, info] : topic_mappings_) {
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
    return topic_mappings_;
}

std::vector<std::string> TopicMap::get_all_topics() const {
    std::vector<std::string> topics;
    topics.reserve(topic_mappings_.size());
    for (const auto& [topic, _] : topic_mappings_) {
        topics.push_back(topic);
    }
    std::sort(topics.begin(), topics.end());
    return topics;
}

std::size_t TopicMap::size() const {
    return topic_mappings_.size();
}

std::string TopicMap::infer_namespace(const std::string& topic_name) {
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

std::string TopicMap::infer_sensor(const std::string& topic_name) {
    std::stringstream stream(topic_name);
    std::string segment;
    std::string last;
    while (std::getline(stream, segment, '/')) {
        if (!segment.empty()) {
            last = segment;
        }
    }
    return last;
}

TopicMap& get_topic_map() {
    static TopicMap map;
    return map;
}

} // namespace core
} // namespace horus

