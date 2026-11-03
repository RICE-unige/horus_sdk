#include "horus/remote_rendering/publisher.hpp"
#include <fstream>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <thread>

using namespace horus::remote_rendering;
using namespace std::chrono_literals;
void check(bool result, const char *message) {
    if (!result)
        throw std::runtime_error(message);
}
int main() {
    rclcpp::init(0, nullptr);
    try {
        auto node = std::make_shared<rclcpp::Node>("native_remote_freshness_peer");
        std::ifstream input(HORUS_REMOTE_PROTOCOL_FIXTURE);
        const auto source = Json::parse(input).at("view");
        for (const std::string mode : {"diagnostic", "h264"}) {
            RemotePublisherOptions options;
            options.mode = mode;
            options.view_topic = "/native_freshness/" + mode + "/view";
            options.stream_topic = "/native_freshness/" + mode + "/image";
            options.status_topic = "/native_freshness/" + mode + "/status";
            options.maximum_view_age_seconds = .15;
            RemotePublisher publisher(options);
            auto poses = node->create_publisher<std_msgs::msg::String>(options.view_topic, 1);
            auto feed = [&](uint64_t sequence, uint64_t revision = 4) {
                auto raw = source;
                raw["view_sequence"] = sequence;
                raw["sample_time_ns"] = sequence * 1000000;
                raw["workspace"]["revision"] = revision;
                std_msgs::msg::String message;
                message.data = raw.dump();
                const auto until = std::chrono::steady_clock::now() + 5s;
                while (std::chrono::steady_clock::now() < until) {
                    poses->publish(message);
                    const auto view = publisher.next_view(20);
                    if (view && view->view_sequence == sequence)
                        return *view;
                }
                throw std::runtime_error("No native view received");
            };
            GpuRenderOptions render{192, 48, .1f, 10.f, 0};
            std::vector<uint8_t> rgb(192 * 48 * 2 * 3, 100);
            std::vector<float> depth(192 * 48 * 2, 2);
            auto send = [&](const ViewState &view, uint64_t sequence, bool idr) {
                auto context = StereoFrameContext::from_view(view, sequence);
                context.codec = mode == "diagnostic" ? "png" : "h264";
                context.marker_height = mode == "diagnostic" ? 0 : 16;
                auto frame = assemble_stereo_frame(rgb, depth, render, context);
                if (mode == "diagnostic")
                    return publisher.publish_diagnostic(frame);
                // Header-only transport-gate fixture, not a codec-valid video sample.
                EncodedH264Frame packet{{0, 0, 0, 1, uint8_t(idr ? 0x65 : 0x41)},
                                        frame.metadata.dump(),
                                        sequence,
                                        sequence,
                                        idr};
                return publisher.publish_h264(packet);
            };
            const auto first = feed(1);
            check(send(first, 1, true), "Initial frame rejected");
            check(!send(first, 1, true), "Duplicate frame accepted");
            check(send(first, 2, false), "Duplicate delivery broke reference chain");
            std::this_thread::sleep_for(250ms);
            const auto fresh = feed(2);
            check(!send(first, 3, false), "Old source view accepted while newer poses arrived");
            if (mode == "h264") {
                check(publisher.take_keyframe_request(), "No recovery request after discard");
                check(!send(fresh, 4, false), "Dependent frame accepted after reference discard");
            }
            check(send(fresh, 5, true), "Fresh recovery frame rejected");
            check(send(fresh, 6, false), "Recovered reference chain rejected");
            const auto restarted = feed(3, 5);
            if (mode == "h264")
                check(!send(restarted, 7, false), "New workspace accepted old reference chain");
            check(send(restarted, 8, true), "Workspace IDR rejected");
            auto future = restarted;
            future.view_sequence = 999;
            check(!send(future, 9, true), "Unknown source sequence accepted");
            check(send(restarted, 10, true), "Fresh view lost after invalid metadata");
            publisher.stop();
            check(publisher.status()["state"] == "stopped", "Publisher did not stop");
        }
        rclcpp::shutdown();
    } catch (const std::exception &e) {
        rclcpp::shutdown();
        std::cerr << e.what() << '\n';
        return 1;
    }
}
