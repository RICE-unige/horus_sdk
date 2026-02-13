#ifndef HORUS_ROBOT_ROBOT_HPP
#define HORUS_ROBOT_ROBOT_HPP

#include "horus/core/types.hpp"
#include "horus/robot/dataviz.hpp"
#include "horus/robot/sensors.hpp"
#include <any>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace horus {
namespace robot {

struct RobotDimensions {
    double length{0.0};
    double width{0.0};
    double height{0.0};
};

class Robot {
public:
    Robot(std::string name, core::RobotType robot_type, std::optional<RobotDimensions> dimensions = std::nullopt);

    const std::string& get_name() const { return name_; }
    core::RobotType get_robot_type() const { return robot_type_; }
    std::string get_type_str() const;

    void add_metadata(const std::string& key, const std::any& value);
    std::optional<std::any> get_metadata(const std::string& key) const;
    const core::Metadata& get_all_metadata() const { return metadata_; }

    void add_sensor(const std::shared_ptr<Sensor>& sensor);
    bool remove_sensor(const std::string& sensor_name);
    std::shared_ptr<Sensor> get_sensor(const std::string& sensor_name) const;
    std::vector<std::shared_ptr<Sensor>> get_sensors_by_type(core::SensorType type) const;
    std::size_t get_sensor_count() const { return sensors_.size(); }
    bool has_sensors() const { return !sensors_.empty(); }
    const std::vector<std::shared_ptr<Sensor>>& get_sensors() const { return sensors_; }

    std::shared_ptr<dataviz::DataViz> create_dataviz(const std::string& dataviz_name = "") const;
    void add_path_planning_to_dataviz(
        dataviz::DataViz& dataviz,
        const std::optional<std::string>& global_path_topic = std::nullopt,
        const std::optional<std::string>& local_path_topic = std::nullopt,
        const std::optional<std::string>& trajectory_topic = std::nullopt) const;
    std::shared_ptr<dataviz::DataViz> create_full_dataviz(
        const std::string& dataviz_name = "",
        const std::optional<std::string>& global_path_topic = std::nullopt,
        const std::optional<std::string>& local_path_topic = std::nullopt,
        const std::optional<std::string>& trajectory_topic = std::nullopt) const;

    std::pair<bool, std::map<std::string, std::any>> register_with_horus(
        std::shared_ptr<dataviz::DataViz> dataviz = nullptr,
        bool keep_alive = true,
        bool show_dashboard = true,
        std::optional<double> workspace_scale = std::nullopt);

    std::pair<bool, std::map<std::string, std::any>> unregister_from_horus();
    bool is_registered_with_horus() const;
    std::optional<std::string> get_horus_id() const;
    std::optional<std::string> get_horus_color() const;

    const std::optional<RobotDimensions>& get_dimensions() const { return dimensions_; }

private:
    std::string name_;
    core::RobotType robot_type_;
    core::Metadata metadata_;
    std::vector<std::shared_ptr<Sensor>> sensors_;
    std::optional<RobotDimensions> dimensions_;
};

std::pair<bool, std::map<std::string, std::any>> register_robots(
    std::vector<Robot*>& robots,
    std::vector<dataviz::DataViz> datavizs = {},
    double timeout_sec = 10.0,
    bool keep_alive = true,
    bool show_dashboard = true,
    std::optional<double> workspace_scale = std::nullopt);

} // namespace robot
} // namespace horus

#endif // HORUS_ROBOT_ROBOT_HPP

