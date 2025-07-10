// SPDX-FileCopyrightText: 2025 RICE Lab, University of Genoa
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

namespace horus_backend
{

class RobotPlugin
{
public:
  virtual ~RobotPlugin() = default;
  virtual void initialize(const std::string & robot_name) = 0;
  virtual bool is_robot_type(const std::string & robot_name) = 0;
  virtual std::string get_robot_type() const = 0;
  virtual std::map<std::string, std::string> get_topic_mappings() const = 0;
  virtual std::map<std::string, double> get_parameters() const = 0;
};

class PluginManager
{
public:
  PluginManager();
  ~PluginManager();

  void load_plugins();
  void register_plugin(
    const std::string & name,
    std::unique_ptr<RobotPlugin> plugin);
  RobotPlugin * get_plugin(const std::string & robot_type);
  std::vector<std::string> get_available_plugins() const;

  // Robot management
  bool connect_robot(
    const std::string & robot_id,
    const std::string & robot_type);
  bool disconnect_robot(const std::string & robot_id);
  bool is_robot_connected(const std::string & robot_id) const;

private:
  std::map<std::string, std::unique_ptr<RobotPlugin>> plugins_;
  std::map<std::string, std::string>
  connected_robots_;      // robot_id -> robot_type

  void load_builtin_plugins();
};

// Built-in plugins
class GenericRobotPlugin : public RobotPlugin
{
public:
  void initialize(const std::string & robot_name) override;
  bool is_robot_type(const std::string & robot_name) override;
  std::string get_robot_type() const override;
  std::map<std::string, std::string> get_topic_mappings() const override;
  std::map<std::string, double> get_parameters() const override;
};

}  // namespace horus_backend
