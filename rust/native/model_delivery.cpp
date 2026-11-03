#include "model_delivery.hpp"
#include "horus/description/compiler.hpp"
#include "horus/teaching/config.hpp"

namespace horus::native {
ModelDelivery::ModelDelivery(rclcpp::Node::SharedPtr node) : node_(std::move(node)) {
    subscription_ = node_->create_subscription<String>("/horus/robot_description/request",
                                                       rclcpp::QoS(32).reliable(),
                                                       [this](String::ConstSharedPtr msg) {
                                                           try {
                                                               request(Json::parse(msg->data));
                                                           } catch (const std::exception &e) {
                                                               std::lock_guard guard(mutex_);
                                                               error_ = e.what();
                                                           }
                                                       });
    timer_ = node_->create_wall_timer(std::chrono::milliseconds(8), [this] {
        try {
            tick();
        } catch (const std::exception &e) {
            std::lock_guard guard(mutex_);
            error_ = e.what();
        }
    });
}
void ModelDelivery::prepare(Json &payload) {
    const auto name = payload.at("robot_name").get<std::string>();
    std::shared_ptr<Model> model;
    if (payload.value("action", "register") != "unregister" &&
        payload.contains("robot_description_manifest")) {
        const auto &manifest = payload.at("robot_description_manifest");
        const auto data = payload.at("robot_description_payload_json").get<std::string>();
        if (data.size() > 256 * 1024 * 1024)
            throw std::length_error("Robot model exceeds delivery byte budget");
        const auto parsed = Json::parse(data);
        if (manifest.at("description_id") != "sha256:" + horus::teaching::sha256(data) ||
            parsed.at("version") != "v2" || !parsed.at("links").is_array() ||
            !parsed.at("joints").is_array())
            throw std::invalid_argument("Invalid/incomplete robot description artifact");
        if (manifest.value("supports_visual_meshes", false) &&
            (parsed.value("visual_links", Json::array()).empty() ||
             parsed.value("mesh_assets", Json::array()).empty()))
            throw std::invalid_argument("Robot visual mesh is missing its link bindings/assets");
        model =
            std::make_shared<Model>(Model{manifest, horus::description::encode_description(data)});
        if (data.size() > 250000)
            payload.erase("robot_description_payload_json");
    }
    std::lock_guard guard(mutex_);
    if (stopped_)
        throw std::runtime_error("Model delivery stopped");
    auto previous = models_.find(name);
    const std::size_t old = previous == models_.end() ? 0 : previous->second->encoded.size();
    if (model && (models_.size() >= 256 && previous == models_.end() ||
                  model->encoded.size() > 512 * 1024 * 1024 - (bytes_ - old)))
        throw std::length_error("Retained robot models exceed delivery budget");
    bytes_ -= old;
    models_.erase(name);
    for (auto it = transfers_.begin(); it != transfers_.end();) {
        if (it->second.name == name)
            it = transfers_.erase(it);
        else
            ++it;
    }
    if (model) {
        bytes_ += model->encoded.size();
        models_[name] = std::move(model);
    }
}
ModelDelivery::Publisher ModelDelivery::publisher(const std::string &topic) {
    auto found = publishers_.find(topic);
    if (found != publishers_.end())
        return found->second;
    if (publishers_.size() >= 96) {
        // Retain active transfers, not every reply topic used since process startup.
        // Temporary transfer handles also own their publishers during construction.
        std::erase_if(publishers_, [](const auto &entry) { return entry.second.use_count() == 1; });
    }
    if (publishers_.size() >= 96)
        throw std::length_error("Too many simultaneous model reply topics");
    auto value =
        node_->create_publisher<String>(topic, rclcpp::QoS(32).reliable().durability_volatile());
    publishers_[topic] = value;
    return value;
}
void ModelDelivery::request(const Json &request) {
    const auto id = request.at("request_id").get<std::string>();
    if (id.empty())
        throw std::invalid_argument("Missing model request id");
    const auto name = request.value("robot_name", "");
    const auto description = request.value("description_id", "");
    std::lock_guard guard(mutex_);
    if (stopped_)
        return;
    auto model = models_.find(name);
    if (model == models_.end() && !description.empty())
        for (auto it = models_.begin(); it != models_.end(); ++it)
            if (it->second->manifest.at("description_id") == description) {
                model = it;
                break;
            }
    if (model == models_.end() ||
        (!description.empty() && model->second->manifest.at("description_id") != description)) {
        error_ = "Requested model is not retained by this SDK";
        return;
    }
    if (transfers_.count(id))
        return;
    if (transfers_.size() >= 8) {
        error_ = "Model delivery busy; retry request";
        return;
    }
    Transfer t;
    t.request = request;
    t.name = model->first;
    t.model = model->second;
    t.chunk = std::clamp(t.model->manifest.value("chunk_size_bytes", 12000), 1024, 64000);
    t.count = (t.model->encoded.size() + t.chunk - 1) / t.chunk;
    t.begin = publisher(request.value("chunk_begin_topic", "/horus/robot_description/chunk_begin"));
    t.item = publisher(request.value("chunk_item_topic", "/horus/robot_description/chunk_item"));
    t.end = publisher(request.value("chunk_end_topic", "/horus/robot_description/chunk_end"));
    t.due = std::chrono::steady_clock::now() + std::chrono::milliseconds(100);
    t.discovery_deadline = t.due + std::chrono::seconds(5);
    transfers_.emplace(id, std::move(t));
    error_.clear();
}
void ModelDelivery::tick() {
    Publisher pub;
    Json message;
    {
        std::lock_guard guard(mutex_);
        if (stopped_ || transfers_.empty())
            return;
        auto it = transfers_.upper_bound(cursor_);
        if (it == transfers_.end())
            it = transfers_.begin();
        cursor_ = it->first;
        auto &t = it->second;
        const auto now = std::chrono::steady_clock::now();
        if (now < t.due)
            return;
        message = {{"request_id", it->first},
                   {"robot_name", t.request.value("robot_name", t.name)},
                   {"description_id", t.model->manifest.at("description_id")}};
        if (!t.begun) {
            if (!t.begin->get_subscription_count() || !t.item->get_subscription_count() ||
                !t.end->get_subscription_count()) {
                if (now >= t.discovery_deadline) {
                    error_ = "Model reply subscribers did not appear; retry request";
                    transfers_.erase(it);
                }
                return;
            }
            pub = t.begin;
            message["session_id"] = t.request.value("session_id", "");
            message["app_id"] = t.request.value("app_id", "");
            message["expected_chunks"] = t.count;
            message["encoding"] = "json+gzip+base64";
            message["ts_unix_ms"] = horus::teaching::unix_ns() / 1000000;
            t.begun = true;
            t.due = now + std::chrono::milliseconds(32);
        } else if (t.next < t.count) {
            pub = t.item;
            message["chunk_index"] = t.next;
            message["total_chunks"] = t.count;
            message["chunk_data"] = t.model->encoded.substr(t.next * t.chunk, t.chunk);
            ++t.next;
            t.due = now + std::chrono::milliseconds(t.next == t.count ? 50 : 8);
        } else {
            pub = t.end;
            message["received_count"] = t.count;
            message["ts_unix_ms"] = horus::teaching::unix_ns() / 1000000;
            transfers_.erase(it);
        }
        ++sent_;
    }
    String wire;
    wire.data = message.dump();
    pub->publish(wire);
}
void ModelDelivery::stop() {
    std::lock_guard guard(mutex_);
    stopped_ = true;
    transfers_.clear();
    publishers_.clear();
    models_.clear();
    bytes_ = 0;
}
ModelDelivery::Json ModelDelivery::status() const {
    std::lock_guard guard(mutex_);
    return {{"retained", models_.size()},
            {"bytes", bytes_},
            {"pending", transfers_.size()},
            {"reply_publishers", publishers_.size()},
            {"sent", sent_},
            {"error", error_}};
}
} // namespace horus::native
