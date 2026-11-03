#include "teach_runtime.h"
#include "core/api.h"
#include "horus/teaching/session.hpp"
#include <atomic>
#include <cstring>
#include <fcntl.h>
#include <filesystem>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/typesupport_helpers.hpp>
#include <rmw/rmw.h>
#include <rosbag2_cpp/message_definitions/local_message_definition_source.hpp>
#include <rosidl_typesupport_introspection_cpp/field_types.hpp>
#include <rosidl_typesupport_introspection_cpp/message_introspection.hpp>
#include <std_msgs/msg/string.hpp>
#include <sys/file.h>
#include <thread>
#include <unistd.h>

namespace {
using namespace horus::teaching;
using Members = rosidl_typesupport_introspection_cpp::MessageMembers;
using Member = rosidl_typesupport_introspection_cpp::MessageMember;
const Member *member(const Members *m, const char *name) {
    for (std::uint32_t i = 0; i < m->member_count_; ++i)
        if (std::strcmp(m->members_[i].name_, name) == 0 && !m->members_[i].is_array_)
            return &m->members_[i];
    return nullptr;
}
const Members *nested(const Member *m) {
    return m && m->members_ && m->members_->data ? static_cast<const Members *>(m->members_->data)
                                                 : nullptr;
}

struct Source {
    std::string id, schema, schema_encoding;
    bool retain_calibration = false;
    const Member *transforms = nullptr, *child_frame = nullptr;
    std::shared_ptr<rcpputils::SharedLibrary> introspection_library, support_library;
    const Members *members = nullptr;
    const rosidl_message_type_support_t *support = nullptr;
    std::optional<std::size_t> sec_offset, nano_offset;
    void *message = nullptr;
    explicit Source(const Json &stream, rosbag2_cpp::LocalMessageDefinitionSource &definitions) {
        id = stream.at("stream_id");
        retain_calibration = retained_calibration(stream);
        std::string type = stream.at("message_type");
        auto definition = definitions.get_full_text(type);
        schema = definition.encoded_message_definition;
        schema_encoding = definition.encoding;
        if (schema.empty() || (schema_encoding != "ros2msg" && schema_encoding != "ros2idl"))
            throw std::runtime_error("Missing complete ROS schema: " + type);
        introspection_library =
            rclcpp::get_typesupport_library(type, "rosidl_typesupport_introspection_cpp");
        auto *handle = rclcpp::get_message_typesupport_handle(
            type, "rosidl_typesupport_introspection_cpp", *introspection_library);
        members = static_cast<const Members *>(handle->data);
        if (retain_calibration && type == "tf2_msgs/msg/TFMessage") {
            for (std::uint32_t i = 0; i < members->member_count_; ++i)
                if (std::strcmp(members->members_[i].name_, "transforms") == 0)
                    transforms = &members->members_[i];
            auto *transform_members = nested(transforms);
            child_frame = transform_members ? member(transform_members, "child_frame_id") : nullptr;
            if (!transforms || !transforms->is_array_ || !transforms->size_function ||
                !transforms->get_const_function || !child_frame ||
                child_frame->type_id_ != rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING)
                throw std::runtime_error("Cannot inspect retained TF children");
        }
        auto *header = member(members, "header");
        auto *header_members = nested(header);
        auto *stamp = header_members ? member(header_members, "stamp") : nullptr;
        auto *stamp_members = nested(stamp);
        auto *sec = stamp_members ? member(stamp_members, "sec") : nullptr;
        auto *nano = stamp_members ? member(stamp_members, "nanosec") : nullptr;
        if (sec && nano &&
            sec->type_id_ == rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32 &&
            nano->type_id_ == rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32) {
            sec_offset = header->offset_ + stamp->offset_ + sec->offset_;
            nano_offset = header->offset_ + stamp->offset_ + nano->offset_;
        }
        if (!sec_offset && !transforms)
            return;
        support_library = rclcpp::get_typesupport_library(type, "rosidl_typesupport_cpp");
        support = rclcpp::get_message_typesupport_handle(type, "rosidl_typesupport_cpp",
                                                         *support_library);
        message = ::operator new(members->size_of_);
        try {
            members->init_function(message, rosidl_runtime_cpp::MessageInitialization::ALL);
        } catch (...) {
            ::operator delete(message);
            message = nullptr;
            throw;
        }
    }
    ~Source() {
        if (message) {
            members->fini_function(message);
            ::operator delete(message);
        }
    }
    std::optional<std::uint64_t> timestamp(const rclcpp::SerializedMessage &serialized) {
        if (!message)
            return {};
        if (rmw_deserialize(&serialized.get_rcl_serialized_message(), support, message) !=
            RMW_RET_OK)
            throw std::runtime_error("Cannot decode source timestamp: " + id);
        if (!sec_offset)
            return {};
        std::int32_t sec;
        std::uint32_t nano;
        std::memcpy(&sec, static_cast<const char *>(message) + *sec_offset, sizeof(sec));
        std::memcpy(&nano, static_cast<const char *>(message) + *nano_offset, sizeof(nano));
        if (sec < 0 || nano >= 1000000000)
            throw std::runtime_error("Invalid ROS source timestamp: " + id);
        return std::uint64_t(sec) * 1000000000 + nano;
    }
    std::vector<std::string> snapshot_keys() const {
        if (!transforms)
            return {"latest"};
        const void *array = static_cast<const char *>(message) + transforms->offset_;
        const auto size = transforms->size_function(array);
        if (size > 2048)
            throw std::runtime_error("Retained TF exceeds 2048 children");
        std::vector<std::string> keys;
        keys.reserve(size);
        for (std::size_t i = 0; i < size; ++i) {
            const auto *element = static_cast<const char *>(transforms->get_const_function(array, i));
            keys.push_back(*reinterpret_cast<const std::string *>(element + child_frame->offset_));
        }
        return keys;
    }
};

class Runtime {
  public:
    TeachSession &session;
    TeachConfig config;
    std::string robot;
    rclcpp::Context::SharedPtr context;
    rclcpp::Node::SharedPtr node;
    std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
    std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher;
    std::thread ros_worker, control_worker;
    std::atomic<bool> stop{false};
    std::mutex error_mutex;
    std::string error;
    int lock_fd = -1;
    Runtime(TeachSession &s, TeachConfig c, std::string name)
        : session(s), config(std::move(c)), robot(std::move(name)) {
        try {
            validate_identifier(robot);
            auto initial = session.state();
            if (initial.at("robot_name") != robot || initial.at("config_hash") != config.digest())
                throw std::invalid_argument(
                    "Teach session and runtime must use the same robot/profile");
            const char *runtime = std::getenv("XDG_RUNTIME_DIR");
            const char *home = std::getenv("HOME");
            if (!runtime && !home)
                throw std::runtime_error("No runtime directory or HOME available");
            auto directory = (runtime ? std::filesystem::path(runtime)
                                      : std::filesystem::path(home) / ".cache") /
                             "horus-teach";
            std::filesystem::create_directories(directory);
            const char *domain = std::getenv("ROS_DOMAIN_ID");
            std::string domain_id = domain ? domain : "0";
            if (domain_id.find_first_not_of("0123456789") != std::string::npos)
                throw std::invalid_argument("Invalid ROS_DOMAIN_ID");
            auto lock_path = directory / (domain_id + "-" + robot + ".lock");
            lock_fd = ::open(lock_path.c_str(), O_CREAT | O_RDWR | O_CLOEXEC, 0600);
            if (lock_fd < 0 || ::flock(lock_fd, LOCK_EX | LOCK_NB))
                throw std::runtime_error("A Teach host for this robot/domain is already running");
            context = std::make_shared<rclcpp::Context>();
            context->init(0, nullptr);
            auto options = rclcpp::NodeOptions().context(context).use_global_arguments(false);
            node = std::make_shared<rclcpp::Node>("horus_teach_" + std::to_string(::getpid()) +
                                                      "_" + new_uuid().substr(0, 8),
                                                  options);
            auto topics = config.registration(robot);
            publisher = node->create_publisher<std_msgs::msg::String>(topics.at("state_topic"),
                                                                      rclcpp::QoS(10));
            subscriptions.push_back(node->create_subscription<std_msgs::msg::String>(
                topics.at("request_topic").get<std::string>(), rclcpp::QoS(10),
                [this](std_msgs::msg::String::ConstSharedPtr msg) {
                    try {
                        if (msg->data.size() > 65536)
                            return;
                        publish(session.request(Json::parse(msg->data)));
                    } catch (const std::exception &e) {
                        record_error(e.what());
                    }
                }));
            subscriptions.push_back(node->create_subscription<std_msgs::msg::String>(
                topics.at("input_topic").get<std::string>(), rclcpp::QoS(1).best_effort(),
                [this](std_msgs::msg::String::ConstSharedPtr msg) {
                    try {
                        if (msg->data.size() > 65536)
                            return;
                        session.input(Json::parse(msg->data));
                    } catch (const std::exception &e) {
                        record_error(e.what());
                    }
                }));
            subscriptions.push_back(node->create_subscription<std_msgs::msg::String>(
                "/horus/multi_operator/control_lease_state",
                rclcpp::QoS(1).reliable().durability_volatile(),
                [this](std_msgs::msg::String::ConstSharedPtr msg) {
                    try {
                        if (msg->data.size() > 1024 * 1024)
                            throw std::invalid_argument("Lease snapshot too large");
                        auto root = Json::parse(msg->data);
                        Json selected = Json::object();
                        for (auto &lease : root.value("leases", Json::array()))
                            if (lease.value("robot_name", "") == robot)
                                selected = lease;
                        session.update_lease(selected.value("holder_app_id", ""),
                                             selected.value("authority_generation", ""),
                                             selected.value("remaining_ms", 0.0) / 1000);
                    } catch (...) {
                        session.update_lease("", "", 0);
                    }
                }));
            rosbag2_cpp::LocalMessageDefinitionSource definitions;
            for (auto &stream : config.to_dict().at("streams")) {
                auto source = std::make_shared<Source>(stream, definitions);
                auto qos = rclcpp::QoS(stream.at("queue_depth").get<std::size_t>());
                if (stream.at("reliability") == "reliable")
                    qos.reliable();
                else
                    qos.best_effort();
                if (stream.at("durability") == "transient_local")
                    qos.transient_local();
                else
                    qos.durability_volatile();
                subscriptions.push_back(node->create_generic_subscription(
                    stream.at("topic"), stream.at("message_type"), qos,
                    [this, source](std::shared_ptr<rclcpp::SerializedMessage> message) {
                        try {
                            session.note_stream(source->id);
                            if (!source->retain_calibration && !session.recording())
                                return;
                            Sample sample;
                            sample.received_unix_ns = unix_ns();
                            sample.received_monotonic_ns = monotonic_ns();
                            sample.stream_id = source->id;
                            sample.source_ns = source->timestamp(*message);
                            sample.schema = source->schema;
                            sample.schema_encoding = source->schema_encoding;
                            const auto &bytes = message->get_rcl_serialized_message();
                            sample.payload.assign(bytes.buffer, bytes.buffer + bytes.buffer_length);
                            session.append(std::move(sample), source->snapshot_keys());
                        } catch (const std::exception &e) {
                            record_error(e.what());
                            stop = true;
                        }
                    }));
            }
            rclcpp::ExecutorOptions exec_options;
            exec_options.context = context;
            executor = std::make_unique<rclcpp::executors::SingleThreadedExecutor>(exec_options);
            executor->add_node(node);
            ros_worker = std::thread([this] {
                try {
                    executor->spin();
                } catch (const std::exception &e) {
                    if (!stop)
                        record_error(e.what());
                }
                stop = true;
            });
            control_worker = std::thread([this] {
                try {
                    double next = 0;
                    while (!stop) {
                        session.tick();
                        if (monotonic_seconds() >= next) {
                            publish(session.state());
                            next = monotonic_seconds() + 0.05;
                        }
                        std::this_thread::sleep_for(std::chrono::milliseconds(10));
                    }
                    session.close();
                } catch (const std::exception &e) {
                    record_error(e.what());
                    stop = true;
                    try {
                        session.close();
                    } catch (...) {
                    }
                }
            });
        } catch (...) {
            shutdown();
            throw;
        }
    }
    ~Runtime() { shutdown(); }
    void record_error(const std::string &message) {
        std::lock_guard guard(error_mutex);
        error = message;
    }
    void publish(const Json &value) {
        if (context && context->is_valid() && !stop) {
            std_msgs::msg::String msg;
            msg.data = value.dump();
            publisher->publish(msg);
        }
    }
    Json status() {
        std::lock_guard guard(error_mutex);
        return {{"running", !stop.load()}, {"error", error}};
    }
    void shutdown() noexcept {
        stop = true;
        if (context)
            try {
                context->shutdown("Teach runtime closed");
            } catch (...) {
            }
        if (executor)
            executor->cancel();
        if (ros_worker.joinable())
            ros_worker.join();
        if (control_worker.joinable())
            control_worker.join();
        subscriptions.clear();
        executor.reset();
        publisher.reset();
        node.reset();
        context.reset();
        if (lock_fd >= 0) {
            ::close(lock_fd);
            lock_fd = -1;
        }
    }
};
char *copy(const std::string &value) {
    auto *out = static_cast<char *>(std::malloc(value.size() + 1));
    if (!out)
        throw std::bad_alloc();
    std::memcpy(out, value.c_str(), value.size() + 1);
    return out;
}
} // namespace
extern "C" {
void *horus_teach_ros_create(void *session, const char *config, const char *robot, char **error) {
    if (error)
        *error = nullptr;
    try {
        if (!session || !config || !robot)
            throw std::invalid_argument("Missing Teach runtime arguments");
        return new Runtime(*static_cast<TeachSession *>(session), TeachConfig(Json::parse(config)),
                           robot);
    } catch (const std::exception &e) {
        if (error)
            try {
                *error = copy(e.what());
            } catch (...) {
            }
        return nullptr;
    } catch (...) {
        return nullptr;
    }
}
char *horus_teach_ros_status(void *runtime) {
    try {
        if (!runtime)
            throw std::invalid_argument("Missing Teach runtime");
        return copy(
            Json{{"ok", true}, {"value", static_cast<Runtime *>(runtime)->status()}}.dump());
    } catch (const std::exception &e) {
        try {
            return copy(Json{{"ok", false}, {"error", e.what()}}.dump());
        } catch (...) {
            return nullptr;
        }
    } catch (...) {
        return nullptr;
    }
}
void horus_teach_ros_destroy(void *runtime) { delete static_cast<Runtime *>(runtime); }
}
