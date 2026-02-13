#include "horus/robot/robot.hpp"

#include "horus/bridge/robot_registry.hpp"
#include <algorithm>
#include <stdexcept>

namespace horus {
namespace robot {

namespace {
std::optional<std::string> any_to_string(const std::any& value) {
    if (value.type() == typeid(std::string)) {
        return std::any_cast<std::string>(value);
    }
    if (value.type() == typeid(const char*)) {
        return std::string(std::any_cast<const char*>(value));
    }
    return std::nullopt;
}

std::optional<bool> any_to_bool(const std::any& value) {
    if (value.type() == typeid(bool)) {
        return std::any_cast<bool>(value);
    }
    if (value.type() == typeid(int)) {
        return std::any_cast<int>(value) != 0;
    }
    if (value.type() == typeid(double)) {
        return std::any_cast<double>(value) != 0.0;
    }
    return std::nullopt;
}
} // namespace

Robot::Robot(std::string name, core::RobotType robot_type, std::optional<RobotDimensions> dimensions)
    : name_(std::move(name)), robot_type_(robot_type), dimensions_(dimensions) {
    if (name_.empty()) {
        throw std::invalid_argument("Robot name cannot be empty");
    }
    if (dimensions_) {
        if (dimensions_->length < 0.0 || dimensions_->width < 0.0 || dimensions_->height < 0.0) {
            throw std::invalid_argument("Robot dimensions must be non-negative");
        }
    }
}

std::string Robot::get_type_str() const {
    return core::robot_type_to_string(robot_type_);
}

void Robot::add_metadata(const std::string& key, const std::any& value) {
    metadata_[key] = value;
}

std::optional<std::any> Robot::get_metadata(const std::string& key) const {
    const auto it = metadata_.find(key);
    if (it == metadata_.end()) {
        return std::nullopt;
    }
    return it->second;
}

void Robot::add_sensor(const std::shared_ptr<Sensor>& sensor) {
    if (!sensor) {
        throw std::invalid_argument("Sensor cannot be null");
    }
    const auto duplicate = std::find_if(sensors_.begin(), sensors_.end(), [&](const auto& existing) {
        return existing->get_name() == sensor->get_name();
    });
    if (duplicate != sensors_.end()) {
        throw std::invalid_argument("Sensor with name '" + sensor->get_name() + "' already exists");
    }
    sensors_.push_back(sensor);
}

bool Robot::remove_sensor(const std::string& sensor_name) {
    const auto before = sensors_.size();
    sensors_.erase(
        std::remove_if(
            sensors_.begin(),
            sensors_.end(),
            [&](const auto& sensor) { return sensor->get_name() == sensor_name; }),
        sensors_.end());
    return before != sensors_.size();
}

std::shared_ptr<Sensor> Robot::get_sensor(const std::string& sensor_name) const {
    const auto it = std::find_if(sensors_.begin(), sensors_.end(), [&](const auto& sensor) {
        return sensor->get_name() == sensor_name;
    });
    if (it == sensors_.end()) {
        return nullptr;
    }
    return *it;
}

std::vector<std::shared_ptr<Sensor>> Robot::get_sensors_by_type(core::SensorType type) const {
    std::vector<std::shared_ptr<Sensor>> result;
    for (const auto& sensor : sensors_) {
        if (sensor->get_sensor_type() == type) {
            result.push_back(sensor);
        }
    }
    return result;
}

std::shared_ptr<dataviz::DataViz> Robot::create_dataviz(const std::string& dataviz_name) const {
    const auto name = dataviz_name.empty() ? (name_ + "_viz") : dataviz_name;
    auto dataviz = std::make_shared<dataviz::DataViz>(name);
    for (const auto& sensor : sensors_) {
        dataviz->add_sensor_visualization(sensor, name_);
    }
    dataviz->add_robot_transform(name_, "/tf", name_ + "_base_link");
    return dataviz;
}

void Robot::add_path_planning_to_dataviz(
    dataviz::DataViz& dataviz,
    const std::optional<std::string>& global_path_topic,
    const std::optional<std::string>& local_path_topic,
    const std::optional<std::string>& trajectory_topic) const {
    if (global_path_topic) {
        dataviz.add_robot_global_path(name_, *global_path_topic, "map");
    }
    if (local_path_topic) {
        dataviz.add_robot_local_path(name_, *local_path_topic, "map");
    }
    if (trajectory_topic) {
        dataviz.add_robot_trajectory(name_, *trajectory_topic, "map");
    }
}

std::shared_ptr<dataviz::DataViz> Robot::create_full_dataviz(
    const std::string& dataviz_name,
    const std::optional<std::string>& global_path_topic,
    const std::optional<std::string>& local_path_topic,
    const std::optional<std::string>& trajectory_topic) const {
    auto dataviz = create_dataviz(dataviz_name);
    add_path_planning_to_dataviz(*dataviz, global_path_topic, local_path_topic, trajectory_topic);
    return dataviz;
}

std::pair<bool, std::map<std::string, std::any>> Robot::register_with_horus(
    std::shared_ptr<dataviz::DataViz> dataviz,
    bool keep_alive,
    bool show_dashboard,
    std::optional<double> workspace_scale) {
    if (!dataviz) {
        dataviz = create_dataviz();
    }
    auto& registry = bridge::get_robot_registry_client();
    auto result =
        registry.register_robot(*this, *dataviz, 10.0, keep_alive, show_dashboard, workspace_scale);
    if (result.first) {
        metadata_["horus_registered"] = true;
    }
    return result;
}

std::pair<bool, std::map<std::string, std::any>> Robot::unregister_from_horus() {
    auto id = get_horus_id();
    if (!id) {
        return {false, {{"error", std::string("Robot not registered with HORUS")}}};
    }
    auto& registry = bridge::get_robot_registry_client();
    auto result = registry.unregister_robot(*id, 5.0);
    if (result.first) {
        metadata_.erase("horus_robot_id");
        metadata_.erase("horus_color");
        metadata_.erase("horus_registered");
        metadata_.erase("horus_topics");
    }
    return result;
}

bool Robot::is_registered_with_horus() const {
    auto value = get_metadata("horus_registered");
    if (!value) {
        return false;
    }
    auto bool_value = any_to_bool(*value);
    return bool_value.value_or(false);
}

std::optional<std::string> Robot::get_horus_id() const {
    auto value = get_metadata("horus_robot_id");
    if (!value) {
        return std::nullopt;
    }
    return any_to_string(*value);
}

std::optional<std::string> Robot::get_horus_color() const {
    auto value = get_metadata("horus_color");
    if (!value) {
        return std::nullopt;
    }
    return any_to_string(*value);
}

std::pair<bool, std::map<std::string, std::any>> register_robots(
    std::vector<Robot*>& robots,
    std::vector<dataviz::DataViz> datavizs,
    double timeout_sec,
    bool keep_alive,
    bool show_dashboard,
    std::optional<double> workspace_scale) {
    auto& registry = bridge::get_robot_registry_client();
    return registry.register_robots(
        robots,
        std::move(datavizs),
        timeout_sec,
        keep_alive,
        show_dashboard,
        workspace_scale);
}

} // namespace robot
} // namespace horus

