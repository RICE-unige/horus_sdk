#ifndef HORUS_CORE_TOPIC_MAP_HPP
#define HORUS_CORE_TOPIC_MAP_HPP

#include "horus/core/types.hpp"
#include <map>
#include <mutex>
#include <nlohmann/json.hpp>
#include <optional>
#include <set>
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
    std::string robot_namespace;
    std::string frame_id;
    Metadata qos_profile;
    double last_seen{};
    bool is_sensor_topic() const;
    bool is_robot_topic() const;
    std::string get_horus_event_topic() const;
};

class TopicMap {
  public:
    TopicMap() = default;
    TopicMap(const TopicMap &);
    TopicMap &operator=(const TopicMap &);
    bool add_topic(const std::string &topic_name, const TopicInfo &topic_info);
    TopicInfo add_topic(const std::string &, TopicType, TopicDirection = TopicDirection::PUBLISH,
                        TopicInfo options = {});
    TopicInfo add_topic(const std::string &, const std::string &ros_type,
                        TopicDirection = TopicDirection::PUBLISH, TopicInfo options = {});
    bool remove_topic(const std::string &topic_name);
    std::optional<TopicInfo> get_topic_info(const std::string &topic_name) const;
    std::vector<std::string> get_topics_by_type(TopicType type) const;
    std::vector<std::string> get_topics_by_namespace(const std::string &namespace_prefix) const;
    std::vector<std::string> get_topics_by_sensor(const std::string &sensor_name) const;
    std::map<std::string, std::size_t> get_topic_stats() const;
    std::unordered_map<std::string, TopicInfo> get_topic_mappings() const;
    std::vector<std::string> get_all_topics() const;
    std::size_t size() const;
    std::optional<TopicInfo> get_topic(const std::string &name) const {
        return get_topic_info(name);
    }
    std::vector<TopicInfo> get_topics_by_robot(const std::string &) const;
    std::vector<TopicInfo> get_sensor_topics(const std::string &robot_namespace = "") const;
    std::set<std::string> get_robot_namespaces() const;
    void update_topic_activity(const std::string &, bool active);
    std::map<std::string, std::string> create_topic_mapping(const std::string &) const;
    nlohmann::json get_stats() const;

  private:
    static TopicInfo prepare_topic(const std::string &, TopicInfo);
    static std::string infer_namespace(const std::string &topic_name);
    static std::string infer_sensor(const std::string &topic_name);
    std::unordered_map<std::string, TopicInfo> topic_mappings_;
    mutable std::mutex mutex_;
};

TopicMap &get_topic_map();

} // namespace core
} // namespace horus

#endif // HORUS_CORE_TOPIC_MAP_HPP
