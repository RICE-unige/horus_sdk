#ifndef HORUS_CORE_TOPIC_MAP_HPP
#define HORUS_CORE_TOPIC_MAP_HPP

#include "horus/core/types.hpp"
#include <map>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace horus {
namespace core {

struct TopicInfo {
    std::string topic_name;
    TopicType topic_type{TopicType::UNKNOWN};
    TopicDirection direction{TopicDirection::BIDIRECTIONAL};
    std::string namespace_prefix;
    std::string sensor_name;
    Metadata metadata;
    bool active{true};
};

class TopicMap {
public:
    bool add_topic(const std::string& topic_name, const TopicInfo& topic_info);
    bool remove_topic(const std::string& topic_name);
    std::optional<TopicInfo> get_topic_info(const std::string& topic_name) const;
    std::vector<std::string> get_topics_by_type(TopicType type) const;
    std::vector<std::string> get_topics_by_namespace(const std::string& namespace_prefix) const;
    std::vector<std::string> get_topics_by_sensor(const std::string& sensor_name) const;
    std::map<std::string, std::size_t> get_topic_stats() const;
    std::unordered_map<std::string, TopicInfo> get_topic_mappings() const;
    std::vector<std::string> get_all_topics() const;
    std::size_t size() const;

private:
    static std::string infer_namespace(const std::string& topic_name);
    static std::string infer_sensor(const std::string& topic_name);
    std::unordered_map<std::string, TopicInfo> topic_mappings_;
};

TopicMap& get_topic_map();

} // namespace core
} // namespace horus

#endif // HORUS_CORE_TOPIC_MAP_HPP

