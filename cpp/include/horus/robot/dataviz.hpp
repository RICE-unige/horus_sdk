#ifndef HORUS_ROBOT_DATAVIZ_HPP
#define HORUS_ROBOT_DATAVIZ_HPP

#include "horus/core/types.hpp"
#include "horus/robot/sensors.hpp"
#include <any>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace horus {
namespace dataviz {

using RenderOptions = std::map<std::string, std::any>;

struct DataSource {
    std::string name;
    core::DataSourceType source_type{core::DataSourceType::SENSOR};
    std::string topic;
    std::string frame_id{"map"};
    std::optional<std::string> robot_name;
    core::Metadata metadata;

    bool is_robot_specific() const { return robot_name.has_value(); }
};

struct VisualizationConfig {
    core::VisualizationType viz_type{core::VisualizationType::MARKERS};
    DataSource data_source;
    std::string display_name;
    bool enabled{true};
    RenderOptions render_options;
    int layer_priority{0};

    bool is_robot_specific() const { return data_source.is_robot_specific(); }
};

class DataViz {
public:
    explicit DataViz(std::string name);

    const std::string& get_name() const { return name_; }

    void add_sensor_visualization(
        const std::shared_ptr<robot::Sensor>& sensor,
        const std::string& robot_name,
        RenderOptions render_options = {});

    void add_robot_transform(
        const std::string& robot_name,
        const std::string& topic,
        const std::string& frame_id = "map",
        RenderOptions render_options = {});

    void add_robot_global_path(
        const std::string& robot_name,
        const std::string& topic,
        const std::string& frame_id = "map",
        RenderOptions render_options = {});

    void add_robot_local_path(
        const std::string& robot_name,
        const std::string& topic,
        const std::string& frame_id = "map",
        RenderOptions render_options = {});

    void add_robot_trajectory(
        const std::string& robot_name,
        const std::string& topic,
        const std::string& frame_id = "map",
        RenderOptions render_options = {});

    void add_occupancy_grid(
        const std::string& topic,
        const std::string& frame_id = "map",
        RenderOptions render_options = {});

    void add_3d_map(
        const std::string& topic,
        const std::string& frame_id = "map",
        RenderOptions render_options = {});

    void add_global_navigation_path(
        const std::string& topic,
        const std::string& frame_id = "map",
        RenderOptions render_options = {});

    void add_navigation_path(
        const std::string& topic,
        const std::string& frame_id = "map",
        RenderOptions render_options = {});

    void add_tf_tree(
        const std::string& topic = "/tf",
        const std::string& frame_id = "map",
        RenderOptions render_options = {});

    std::vector<VisualizationConfig> get_robot_visualizations(const std::string& robot_name) const;
    std::vector<VisualizationConfig> get_global_visualizations() const;
    std::vector<VisualizationConfig> get_visualizations_by_type(core::VisualizationType type) const;
    std::vector<VisualizationConfig> get_enabled_visualizations() const;

    void enable_robot_visualizations(const std::string& robot_name);
    void disable_robot_visualizations(const std::string& robot_name);
    std::size_t remove_robot_visualizations(const std::string& robot_name);

    std::map<std::string, std::any> get_summary() const;
    const std::vector<VisualizationConfig>& get_visualizations() const { return visualizations_; }

private:
    static std::string robot_color_hex(const std::string& robot_name);
    static std::string shifted_color_hex(const std::string& color, int delta);
    static std::optional<std::string> any_to_string(const std::any& value);
    static std::optional<double> any_to_double(const std::any& value);
    static core::VisualizationType viz_type_for_sensor(core::SensorType sensor_type);

    void add_or_update_visualization(const VisualizationConfig& visualization);

    std::string name_;
    std::vector<VisualizationConfig> visualizations_;
};

} // namespace dataviz

namespace robot {
using DataViz = horus::dataviz::DataViz;
using DataSource = horus::dataviz::DataSource;
using VisualizationConfig = horus::dataviz::VisualizationConfig;
using RenderOptions = horus::dataviz::RenderOptions;
} // namespace robot

} // namespace horus

#endif // HORUS_ROBOT_DATAVIZ_HPP

