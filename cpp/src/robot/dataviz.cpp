#include "horus/robot/dataviz.hpp"

#include <algorithm>
#include <cctype>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace horus {
namespace dataviz {

namespace {

std::string to_lower(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    return value;
}

std::string ensure_hash(std::string hex) {
    if (!hex.empty() && hex.front() != '#') {
        hex.insert(hex.begin(), '#');
    }
    return hex;
}

std::string hex_component(int value) {
    value = std::clamp(value, 0, 255);
    std::ostringstream stream;
    stream << std::hex << std::setfill('0') << std::setw(2) << std::uppercase << value;
    return stream.str();
}

std::string brighten_hex(const std::string& color_hex, int delta) {
    if (color_hex.size() != 7 || color_hex[0] != '#') {
        return color_hex;
    }
    const auto parse = [](const std::string& source) -> int {
        return std::stoi(source, nullptr, 16);
    };
    int r = parse(color_hex.substr(1, 2));
    int g = parse(color_hex.substr(3, 2));
    int b = parse(color_hex.substr(5, 2));
    r = std::clamp(r + delta, 0, 255);
    g = std::clamp(g + delta, 0, 255);
    b = std::clamp(b + delta, 0, 255);
    return "#" + hex_component(r) + hex_component(g) + hex_component(b);
}

template <typename T>
std::optional<T> any_cast_optional(const std::any& value) {
    if (value.type() == typeid(T)) {
        return std::any_cast<T>(value);
    }
    return std::nullopt;
}

} // namespace

DataViz::DataViz(std::string name) : name_(std::move(name)) {
    if (name_.empty()) {
        throw std::invalid_argument("DataViz name cannot be empty");
    }
}

void DataViz::add_sensor_visualization(
    const std::shared_ptr<robot::Sensor>& sensor,
    const std::string& robot_name,
    RenderOptions render_options) {
    if (!sensor) {
        return;
    }

    const auto viz_type = viz_type_for_sensor(sensor->get_sensor_type());
    if (render_options.find("color") == render_options.end()) {
        const auto base_color = robot_color_hex(robot_name);
        if (viz_type == core::VisualizationType::LASER_SCAN) {
            render_options["color"] = base_color;
            render_options["alpha"] = 0.8;
        } else {
            render_options["color"] = base_color;
        }
    }

    DataSource source;
    source.name = sensor->get_name();
    source.source_type = core::DataSourceType::SENSOR;
    source.topic = sensor->get_topic();
    source.frame_id = sensor->get_frame_id();
    source.robot_name = robot_name;
    source.metadata = sensor->get_all_metadata();

    VisualizationConfig visualization;
    visualization.viz_type = viz_type;
    visualization.data_source = std::move(source);
    visualization.enabled = true;
    visualization.render_options = std::move(render_options);
    visualization.layer_priority = 0;
    visualization.display_name = robot_name + ":" + sensor->get_name() + " (" +
                                 core::visualization_type_to_string(viz_type) + ")";
    add_or_update_visualization(visualization);
}

void DataViz::add_robot_transform(
    const std::string& robot_name,
    const std::string& topic,
    const std::string& frame_id,
    RenderOptions render_options) {
    if (render_options.find("color") == render_options.end()) {
        render_options["color"] = robot_color_hex(robot_name);
    }
    VisualizationConfig visualization;
    visualization.viz_type = core::VisualizationType::COORDINATE_AXES;
    visualization.display_name = robot_name + ":transform";
    visualization.enabled = true;
    visualization.render_options = std::move(render_options);
    visualization.layer_priority = 0;
    visualization.data_source = DataSource{
        robot_name + "_transform",
        core::DataSourceType::ROBOT_TRANSFORM,
        topic,
        frame_id,
        robot_name,
        {}
    };
    add_or_update_visualization(visualization);
}

void DataViz::add_robot_global_path(
    const std::string& robot_name,
    const std::string& topic,
    const std::string& frame_id,
    RenderOptions render_options) {
    if (render_options.find("color") == render_options.end()) {
        render_options["color"] = robot_color_hex(robot_name);
        render_options["alpha"] = 0.9;
        render_options["line_width"] = 3;
    }
    VisualizationConfig visualization;
    visualization.viz_type = core::VisualizationType::PATH;
    visualization.display_name = robot_name + ":global_path";
    visualization.enabled = true;
    visualization.render_options = std::move(render_options);
    visualization.layer_priority = 3;
    visualization.data_source = DataSource{
        robot_name + "_global_path",
        core::DataSourceType::ROBOT_GLOBAL_PATH,
        topic,
        frame_id,
        robot_name,
        {}
    };
    add_or_update_visualization(visualization);
}

void DataViz::add_robot_local_path(
    const std::string& robot_name,
    const std::string& topic,
    const std::string& frame_id,
    RenderOptions render_options) {
    if (render_options.find("color") == render_options.end()) {
        const auto color = shifted_color_hex(robot_color_hex(robot_name), 48);
        render_options["color"] = color;
        render_options["alpha"] = 0.9;
        render_options["line_width"] = 2;
        render_options["line_style"] = std::string("dashed");
    }
    VisualizationConfig visualization;
    visualization.viz_type = core::VisualizationType::PATH;
    visualization.display_name = robot_name + ":local_path";
    visualization.enabled = true;
    visualization.render_options = std::move(render_options);
    visualization.layer_priority = 4;
    visualization.data_source = DataSource{
        robot_name + "_local_path",
        core::DataSourceType::ROBOT_LOCAL_PATH,
        topic,
        frame_id,
        robot_name,
        {}
    };
    add_or_update_visualization(visualization);
}

void DataViz::add_robot_trajectory(
    const std::string& robot_name,
    const std::string& topic,
    const std::string& frame_id,
    RenderOptions render_options) {
    if (render_options.find("color") == render_options.end()) {
        render_options["color"] = robot_color_hex(robot_name);
        render_options["alpha"] = 0.5;
        render_options["line_width"] = 1;
    }
    VisualizationConfig visualization;
    visualization.viz_type = core::VisualizationType::TRAJECTORY;
    visualization.display_name = robot_name + ":trajectory";
    visualization.enabled = true;
    visualization.render_options = std::move(render_options);
    visualization.layer_priority = 1;
    visualization.data_source = DataSource{
        robot_name + "_trajectory",
        core::DataSourceType::ROBOT_TRAJECTORY,
        topic,
        frame_id,
        robot_name,
        {}
    };
    add_or_update_visualization(visualization);
}

void DataViz::add_occupancy_grid(
    const std::string& topic,
    const std::string& frame_id,
    RenderOptions render_options) {
    VisualizationConfig visualization;
    visualization.viz_type = core::VisualizationType::OCCUPANCY_GRID;
    visualization.display_name = "occupancy_grid";
    visualization.enabled = true;
    visualization.render_options = std::move(render_options);
    visualization.layer_priority = -10;
    visualization.data_source = DataSource{
        "occupancy_grid",
        core::DataSourceType::OCCUPANCY_GRID,
        topic,
        frame_id,
        std::nullopt,
        {}
    };
    add_or_update_visualization(visualization);
}

void DataViz::add_3d_map(const std::string& topic, const std::string& frame_id, RenderOptions render_options) {
    VisualizationConfig visualization;
    visualization.viz_type = core::VisualizationType::POINT_CLOUD;
    visualization.display_name = "map_3d";
    visualization.enabled = true;
    visualization.render_options = std::move(render_options);
    visualization.layer_priority = -5;
    visualization.data_source = DataSource{
        "map_3d",
        core::DataSourceType::MAP_3D,
        topic,
        frame_id,
        std::nullopt,
        {}
    };
    add_or_update_visualization(visualization);
}

void DataViz::add_global_navigation_path(
    const std::string& topic,
    const std::string& frame_id,
    RenderOptions render_options) {
    if (render_options.find("color") == render_options.end()) {
        render_options["color"] = std::string("#00FF00");
    }
    if (render_options.find("line_width") == render_options.end()) {
        render_options["line_width"] = 4;
    }
    if (render_options.find("alpha") == render_options.end()) {
        render_options["alpha"] = 0.8;
    }

    VisualizationConfig visualization;
    visualization.viz_type = core::VisualizationType::PATH;
    visualization.display_name = "global_navigation_path";
    visualization.enabled = true;
    visualization.render_options = std::move(render_options);
    visualization.layer_priority = 5;
    visualization.data_source = DataSource{
        "global_navigation_path",
        core::DataSourceType::GLOBAL_NAVIGATION_PATH,
        topic,
        frame_id,
        std::nullopt,
        {}
    };
    add_or_update_visualization(visualization);
}

void DataViz::add_navigation_path(
    const std::string& topic,
    const std::string& frame_id,
    RenderOptions render_options) {
    add_global_navigation_path(topic, frame_id, std::move(render_options));
}

void DataViz::add_tf_tree(const std::string& topic, const std::string& frame_id, RenderOptions render_options) {
    VisualizationConfig visualization;
    visualization.viz_type = core::VisualizationType::TRANSFORM_TREE;
    visualization.display_name = "tf_tree";
    visualization.enabled = true;
    visualization.render_options = std::move(render_options);
    visualization.layer_priority = 0;
    visualization.data_source = DataSource{
        "tf_tree",
        core::DataSourceType::TF_TREE,
        topic,
        frame_id,
        std::nullopt,
        {}
    };
    add_or_update_visualization(visualization);
}

std::vector<VisualizationConfig> DataViz::get_robot_visualizations(const std::string& robot_name) const {
    std::vector<VisualizationConfig> result;
    for (const auto& visualization : visualizations_) {
        if (visualization.data_source.robot_name == robot_name) {
            result.push_back(visualization);
        }
    }
    return result;
}

std::vector<VisualizationConfig> DataViz::get_global_visualizations() const {
    std::vector<VisualizationConfig> result;
    for (const auto& visualization : visualizations_) {
        if (!visualization.is_robot_specific()) {
            result.push_back(visualization);
        }
    }
    return result;
}

std::vector<VisualizationConfig> DataViz::get_visualizations_by_type(core::VisualizationType type) const {
    std::vector<VisualizationConfig> result;
    for (const auto& visualization : visualizations_) {
        if (visualization.viz_type == type) {
            result.push_back(visualization);
        }
    }
    return result;
}

std::vector<VisualizationConfig> DataViz::get_enabled_visualizations() const {
    std::vector<VisualizationConfig> result;
    for (const auto& visualization : visualizations_) {
        if (visualization.enabled) {
            result.push_back(visualization);
        }
    }
    std::sort(result.begin(), result.end(), [](const auto& left, const auto& right) {
        return left.layer_priority > right.layer_priority;
    });
    return result;
}

void DataViz::enable_robot_visualizations(const std::string& robot_name) {
    for (auto& visualization : visualizations_) {
        if (visualization.data_source.robot_name == robot_name) {
            visualization.enabled = true;
        }
    }
}

void DataViz::disable_robot_visualizations(const std::string& robot_name) {
    for (auto& visualization : visualizations_) {
        if (visualization.data_source.robot_name == robot_name) {
            visualization.enabled = false;
        }
    }
}

std::size_t DataViz::remove_robot_visualizations(const std::string& robot_name) {
    const auto before = visualizations_.size();
    visualizations_.erase(
        std::remove_if(
            visualizations_.begin(),
            visualizations_.end(),
            [&](const VisualizationConfig& visualization) {
                return visualization.data_source.robot_name == robot_name;
            }),
        visualizations_.end());
    return before - visualizations_.size();
}

std::map<std::string, std::any> DataViz::get_summary() const {
    std::map<std::string, std::size_t> by_robot;
    std::map<std::string, std::size_t> by_type;
    std::map<std::string, std::size_t> by_source;

    std::size_t robot_specific = 0;
    std::size_t global = 0;
    for (const auto& visualization : visualizations_) {
        const auto type_str = core::visualization_type_to_string(visualization.viz_type);
        const auto source_str = core::data_source_type_to_string(visualization.data_source.source_type);
        by_type[type_str]++;
        by_source[source_str]++;
        if (visualization.is_robot_specific()) {
            robot_specific++;
            by_robot[*visualization.data_source.robot_name]++;
        } else {
            global++;
        }
    }

    std::map<std::string, std::any> summary;
    summary["name"] = name_;
    summary["total_visualizations"] = visualizations_.size();
    summary["enabled_visualizations"] = get_enabled_visualizations().size();
    summary["robot_specific"] = robot_specific;
    summary["global"] = global;
    summary["by_robot"] = by_robot;
    summary["by_type"] = by_type;
    summary["by_data_source"] = by_source;
    return summary;
}

std::string DataViz::robot_color_hex(const std::string& robot_name) {
    static const std::vector<std::string> palette = {
        "#FF0000", "#00FF00", "#0000FF", "#FF7F00", "#FF00FF",
        "#00FFFF", "#FFFF00", "#FF007F", "#7F00FF", "#00FF7F"
    };
    std::hash<std::string> hasher;
    return palette[hasher(robot_name) % palette.size()];
}

std::string DataViz::shifted_color_hex(const std::string& color, int delta) {
    return brighten_hex(ensure_hash(color), delta);
}

std::optional<std::string> DataViz::any_to_string(const std::any& value) {
    if (value.type() == typeid(std::string)) {
        return std::any_cast<std::string>(value);
    }
    if (value.type() == typeid(const char*)) {
        return std::string(std::any_cast<const char*>(value));
    }
    return std::nullopt;
}

std::optional<double> DataViz::any_to_double(const std::any& value) {
    if (value.type() == typeid(double)) {
        return std::any_cast<double>(value);
    }
    if (value.type() == typeid(float)) {
        return static_cast<double>(std::any_cast<float>(value));
    }
    if (value.type() == typeid(int)) {
        return static_cast<double>(std::any_cast<int>(value));
    }
    return std::nullopt;
}

core::VisualizationType DataViz::viz_type_for_sensor(core::SensorType sensor_type) {
    switch (sensor_type) {
        case core::SensorType::CAMERA:
            return core::VisualizationType::CAMERA_FEED;
        case core::SensorType::LASER_SCAN:
            return core::VisualizationType::LASER_SCAN;
        case core::SensorType::LIDAR_3D:
            return core::VisualizationType::POINT_CLOUD;
        default:
            return core::VisualizationType::MARKERS;
    }
}

void DataViz::add_or_update_visualization(const VisualizationConfig& visualization) {
    auto it = std::find_if(
        visualizations_.begin(),
        visualizations_.end(),
        [&](const VisualizationConfig& existing) {
            return existing.data_source.name == visualization.data_source.name &&
                   existing.viz_type == visualization.viz_type;
        });
    if (it != visualizations_.end()) {
        *it = visualization;
    } else {
        visualizations_.push_back(visualization);
    }
}

} // namespace dataviz
} // namespace horus

