// SPDX-FileCopyrightText: 2025 RICE Lab, University of Genoa
// SPDX-License-Identifier: Apache-2.0

#include "horus_backend/plugin_manager.hpp"

#include <iostream>

namespace horus_backend
{

PluginManager::PluginManager() {load_builtin_plugins();}

PluginManager::~PluginManager() = default;

void PluginManager::load_plugins()
{
  // Load built-in plugins
  load_builtin_plugins();

  std::cout << "Loaded " << plugins_.size() << " robot plugins" << std::endl;
  for (const auto & plugin : plugins_) {
    std::cout << "  - " << plugin.first << " ("
              << plugin.second->get_robot_type() << ")" << std::endl;
  }
}

void PluginManager::register_plugin(
  const std::string & name,
  std::unique_ptr<RobotPlugin> plugin)
{
  plugins_[name] = std::move(plugin);
  std::cout << "Registered plugin: " << name << std::endl;
}

RobotPlugin * PluginManager::get_plugin(const std::string & robot_type)
{
  auto it = plugins_.find(robot_type);
  if (it != plugins_.end()) {
    return it->second.get();
  }

  // Try to find by robot type
  for (const auto & plugin : plugins_) {
    if (plugin.second->get_robot_type() == robot_type) {
      return plugin.second.get();
    }
  }

  return nullptr;
}

std::vector<std::string> PluginManager::get_available_plugins() const
{
  std::vector<std::string> plugin_names;
  for (const auto & plugin : plugins_) {
    plugin_names.push_back(plugin.first);
  }
  return plugin_names;
}

bool PluginManager::connect_robot(
  const std::string & robot_id,
  const std::string & robot_type)
{
  RobotPlugin * plugin = get_plugin(robot_type);
  if (!plugin) {
    std::cerr << "No plugin found for robot type: " << robot_type << std::endl;
    return false;
  }

  try {
    plugin->initialize(robot_id);
    connected_robots_[robot_id] = robot_type;
    std::cout << "Connected robot: " << robot_id << " (type: " << robot_type
              << ")" << std::endl;
    return true;
  } catch (const std::exception & e) {
    std::cerr << "Failed to connect robot " << robot_id << ": " << e.what()
              << std::endl;
    return false;
  }
}

bool PluginManager::disconnect_robot(const std::string & robot_id)
{
  auto it = connected_robots_.find(robot_id);
  if (it != connected_robots_.end()) {
    connected_robots_.erase(it);
    std::cout << "Disconnected robot: " << robot_id << std::endl;
    return true;
  }
  return false;
}

bool PluginManager::is_robot_connected(const std::string & robot_id) const
{
  return connected_robots_.find(robot_id) != connected_robots_.end();
}

void PluginManager::load_builtin_plugins()
{
  // Register generic robot plugin
  register_plugin("generic", std::make_unique<GenericRobotPlugin>());
}

// GenericRobotPlugin implementation
void GenericRobotPlugin::initialize(const std::string & robot_name)
{
  std::cout << "Initializing generic robot plugin for: " << robot_name
            << std::endl;
}

bool GenericRobotPlugin::is_robot_type(const std::string & /* robot_name */)
{
  // Generic plugin can handle any robot
  return true;
}

std::string GenericRobotPlugin::get_robot_type() const {return "generic";}

std::map<std::string, std::string> GenericRobotPlugin::get_topic_mappings()
const
{
  return {{"cmd_vel", "/cmd_vel"},
    {"odom", "/odom"},
    {"scan", "/scan"},
    {"battery", "/battery_state"}};
}

std::map<std::string, double> GenericRobotPlugin::get_parameters() const
{
  return {
    {"max_linear_vel", 1.0}, {"max_angular_vel", 1.0}, {"wheel_radius", 0.1}};
}

}  // namespace horus_backend
