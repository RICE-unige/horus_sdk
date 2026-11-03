#include "horus/core/topic_map.hpp"
#include <cassert>
#include <thread>

using namespace horus::core;

int main() {
    TopicMap map;
    const auto camera = map.add_topic("/robot1/camera/image", "sensor_msgs/msg/Image");
    assert(camera.get_horus_event_topic() == "robot1.sensor.camera");
    assert(camera.is_sensor_topic() && camera.is_robot_topic());
    assert(camera.last_seen > 0);
    TopicInfo options;
    options.robot_namespace = "g1";
    options.frame_id = "lidar_link";
    options.qos_profile["reliability"] = std::string("best_effort");
    const auto lidar =
        map.add_topic("/points", TopicType::POINT_CLOUD, TopicDirection::SUBSCRIBE, options);
    assert(lidar.robot_namespace == "g1" && lidar.frame_id == "lidar_link");
    assert(map.get_topics_by_robot("g1").size() == 1);
    assert(map.get_sensor_topics().size() == 2);
    assert(map.get_sensor_topics("robot1").size() == 1);
    assert(map.create_topic_mapping("robot1").at("camera") == camera.topic_name);
    assert(map.add_topic("/tf_static", TopicType::TF_STATIC).get_horus_event_topic() ==
           "global.tf");
    assert(map.add_topic("/map", "nav_msgs/msg/OccupancyGrid").get_horus_event_topic() ==
           "global.map");
    map.update_topic_activity(camera.topic_name, false);
    map.update_topic_activity("/absent", false);
    const auto stats = map.get_stats();
    assert(stats["total_topics"] == 4 && stats["active_topics"] == 3);
    assert(stats["robot_count"] == 2 && stats["sensor_topics"] == 2);
    assert(stats["topic_types"]["tf2_msgs/TFMessage"] == 1);
    assert(!map.add_topic("", options));
    bool rejected = false;
    try {
        map.add_topic("", TopicType::UNKNOWN);
    } catch (const std::invalid_argument &) {
        rejected = true;
    }
    assert(rejected);
    TopicMap copy = map;
    assert(map.remove_topic("/points"));
    assert(!map.remove_topic("/points"));
    assert(map.get_robot_namespaces().size() == 1);
    assert(copy.get_robot_namespaces().size() == 2);
    copy = map;
    assert(copy.size() == 3);
    // Insertion must not dereference a record another thread has just removed.
    std::thread writer([&] {
        for (int i = 0; i < 2000; ++i) {
            const auto info = map.add_topic("/robot2/scan", TopicType::LASER_SCAN);
            assert(info.topic_name == "/robot2/scan");
        }
    });
    std::thread remover([&] {
        for (int i = 0; i < 2000; ++i) {
            map.remove_topic("/robot2/scan");
            (void)map.get_stats();
            (void)map.get_topic_mappings();
        }
    });
    writer.join();
    remover.join();
}
