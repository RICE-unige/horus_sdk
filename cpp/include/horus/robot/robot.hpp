#ifndef HORUS_ROBOT_ROBOT_HPP
#define HORUS_ROBOT_ROBOT_HPP

#include "horus/core/types.hpp"
#include "horus/robot/sensors.hpp"
#include <string>
#include <vector>
#include <memory>
#include <optional>

namespace horus {

// Forward declarations
namespace dataviz { class DataViz; }

namespace robot {

/**
 * @brief Base robot class with type and identification
 */
class Robot {
public:
    Robot(const std::string& name, core::RobotType robot_type);
    
    // Getters
    std::string get_name() const { return name_; }
    core::RobotType get_robot_type() const { return robot_type_; }
    std::string get_type_str() const;
    
    // Metadata management
    void add_metadata(const std::string& key, const std::any& value);
    std::optional<std::any> get_metadata(const std::string& key) const;
    
    // Sensor management
    void add_sensor(std::shared_ptr<Sensor> sensor);
    bool remove_sensor(const std::string& sensor_name);
    std::shared_ptr<Sensor> get_sensor(const std::string& sensor_name) const;
    std::vector<std::shared_ptr<Sensor>> get_sensors_by_type(core::SensorType type) const;
    size_t get_sensor_count() const { return sensors_.size(); }
    bool has_sensors() const { return !sensors_.empty(); }
    
    // DataViz creation
    std::shared_ptr<dataviz::DataViz> create_dataviz(const std::string& dataviz_name = "");
    
    // HORUS registration
    std::pair<bool, std::map<std::string, std::any>> register_with_horus(
        std::shared_ptr<dataviz::DataViz> dataviz = nullptr);
    std::pair<bool, std::map<std::string, std::any>> unregister_from_horus();
    bool is_registered_with_horus() const;
    std::optional<std::string> get_horus_id() const;
    std::optional<std::string> get_horus_color() const;
    
private:
    std::string name_;
    core::RobotType robot_type_;
    core::Metadata metadata_;
    std::vector<std::shared_ptr<Sensor>> sensors_;
};

} // namespace robot
} // namespace horus

#endif // HORUS_ROBOT_ROBOT_HPP
