#include <cassert>
#include <functional>
#include <horus/tf.hpp>
#include <limits>
#include <vector>
#ifdef HORUS_ROS2_ENABLED
#include <rclcpp/rclcpp.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <thread>
#endif

int main() {
#ifdef HORUS_ROS2_ENABLED
    using namespace horus::tf;
    using Message = tf2_msgs::msg::TFMessage;
    rclcpp::init(0, nullptr);
    auto node = std::make_shared<rclcpp::Node>("tf_test_peer");
    std::vector<Message> received, retained;
    PublisherOptions options;
    options.topic = "/native_test/tf";
    options.static_topic = "/native_test/tf_static";
    auto sub = node->create_subscription<Message>(
        options.topic, 100, [&](Message::ConstSharedPtr m) { received.push_back(*m); });
    const auto wait = [&](const std::function<bool()> &ready) {
        const auto end = std::chrono::steady_clock::now() + std::chrono::seconds(4);
        do {
            rclcpp::spin_some(node);
            if (ready())
                return true;
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        } while (std::chrono::steady_clock::now() < end);
        return false;
    };
    {
        Publisher publisher(options);
        Transform first{"map", "robot/base", {1, 2, 3}, {0, 0, 0, 1}, 42};
        assert(wait(
            [&] { return publisher.subscriber_count() == 1 && sub->get_publisher_count() == 1; }));
        publisher.publish(std::span(&first, 1));
        assert(wait([&] { return received.size() == 1; }));
        assert(received[0].transforms[0].header.stamp.nanosec == 42);
        assert(received[0].transforms[0].transform.translation.y == 2);
        Transform fixed{"robot/base", "robot/camera", {0.3, 0, 0.5}, {0, 0, 0, 1}, 0};
        publisher.publish(std::span(&fixed, 1), true);
        fixed.child = "robot/lidar";
        fixed.translation = {0, 0, 0.4};
        publisher.publish(std::span(&fixed, 1), true);
        auto late = node->create_subscription<Message>(
            options.static_topic, rclcpp::QoS(1).reliable().transient_local(),
            [&](Message::ConstSharedPtr m) { retained.push_back(*m); });
        assert(wait([&] { return !retained.empty(); }));
        assert(retained.back().transforms.size() == 2);
        for (const auto &t : retained.back().transforms) {
            assert(t.header.frame_id == "robot/base");
            assert(t.header.stamp.sec == 0 && t.header.stamp.nanosec == 0);
        }
        std::vector<Transform> invalid;
        auto bad = first;
        bad.child = first.parent;
        invalid.push_back(bad);
        bad = first;
        bad.rotation = {0, 0, 0, 0};
        invalid.push_back(bad);
        bad = first;
        bad.rotation[0] = std::numeric_limits<double>::quiet_NaN();
        invalid.push_back(bad);
        bad = first;
        bad.translation[1] = std::numeric_limits<double>::infinity();
        invalid.push_back(bad);
        bad = first;
        bad.parent = "/map";
        invalid.push_back(bad);
        bad = first;
        bad.child = "robot base";
        invalid.push_back(bad);
        bad = first;
        bad.child = std::string("base\0other", 10);
        invalid.push_back(bad);
        bad = first;
        bad.stamp_ns = -1;
        invalid.push_back(bad);
        bad = first;
        bad.stamp_ns = std::numeric_limits<int64_t>::max();
        invalid.push_back(bad);
        for (const auto &value : invalid) {
            auto good = first;
            good.child = "not_published";
            std::array batch{good, value};
            bool threw = false;
            try {
                publisher.publish(batch);
            } catch (const std::exception &) {
                threw = true;
            }
            assert(threw);
        }
        bool duplicate_rejected = false;
        try {
            publisher.publish(std::array{first, first});
        } catch (const std::exception &) {
            duplicate_rejected = true;
        }
        assert(duplicate_rejected);
        first.stamp_ns.reset();
        publisher.publish(std::span(&first, 1));
        assert(wait([&] { return received.size() >= 2; }));
        assert(received.size() == 2); // No partial batch escaped validation.
        assert(received.back().transforms[0].header.stamp.sec > 1'700'000'000);
    }
    assert(rclcpp::ok()); // Private publisher destruction must not stop the caller.
    Publisher survivor(options);
    Transform after{"map", "after_destroy"};
    assert(
        wait([&] { return survivor.subscriber_count() == 1 && sub->get_publisher_count() == 1; }));
    survivor.publish(std::span(&after, 1));
    assert(wait([&] { return received.size() == 3; }));
    rclcpp::shutdown();
#else
    bool threw = false;
    try {
        horus::tf::Publisher unavailable;
    } catch (const std::exception &) {
        threw = true;
    }
    assert(threw);
#endif
}
