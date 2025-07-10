// SPDX-FileCopyrightText: 2025 RICE Lab, University of Genoa
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "horus_backend/plugin_manager.hpp"
#include "horus_backend/tcp_server.hpp"
#include "horus_interfaces/msg/robot_command.hpp"
#include "horus_interfaces/msg/robot_config.hpp"
#include "horus_interfaces/msg/robot_status.hpp"
#include "horus_interfaces/srv/connect_robot.hpp"
#include "horus_interfaces/srv/execute_command.hpp"
#include "horus_interfaces/srv/register_robot.hpp"
#include "horus_interfaces/srv/unregister_robot.hpp"

namespace horus_backend
{

class BackendNode : public rclcpp::Node
{
public:
  BackendNode();
  ~BackendNode();

  void initialize();
  void shutdown();

private:
  // Core components
  std::unique_ptr<TcpServer> tcp_server_;
  std::unique_ptr<PluginManager> plugin_manager_;

  // ROS2 Publishers
  rclcpp::Publisher<horus_interfaces::msg::RobotCommand>::SharedPtr
    command_publisher_;
  rclcpp::Publisher<horus_interfaces::msg::RobotStatus>::SharedPtr
    status_publisher_;

  // ROS2 Subscribers
  rclcpp::Subscription<horus_interfaces::msg::RobotStatus>::SharedPtr
    status_subscriber_;

  // ROS2 Services
  rclcpp::Service<horus_interfaces::srv::ConnectRobot>::SharedPtr
    connect_service_;
  rclcpp::Service<horus_interfaces::srv::ExecuteCommand>::SharedPtr
    execute_service_;
  rclcpp::Service<horus_interfaces::srv::RegisterRobot>::SharedPtr
    register_service_;
  rclcpp::Service<horus_interfaces::srv::UnregisterRobot>::SharedPtr
    unregister_service_;

  // Parameters
  int tcp_port_;
  int unity_tcp_port_;
  std::string log_level_;

  // Callbacks
  void status_callback(const horus_interfaces::msg::RobotStatus::SharedPtr msg);
  void connect_robot_callback(
    const std::shared_ptr<horus_interfaces::srv::ConnectRobot::Request>
    request,
    std::shared_ptr<horus_interfaces::srv::ConnectRobot::Response> response);
  void execute_command_callback(
    const std::shared_ptr<horus_interfaces::srv::ExecuteCommand::Request>
    request,
    std::shared_ptr<horus_interfaces::srv::ExecuteCommand::Response>
    response);
  void register_robot_callback(
    const std::shared_ptr<horus_interfaces::srv::RegisterRobot::Request>
    request,
    std::shared_ptr<horus_interfaces::srv::RegisterRobot::Response> response);
  void unregister_robot_callback(
    const std::shared_ptr<horus_interfaces::srv::UnregisterRobot::Request>
    request,
    std::shared_ptr<horus_interfaces::srv::UnregisterRobot::Response>
    response);

  // Internal methods
  void setup_parameters();
  void setup_publishers();
  void setup_subscribers();
  void setup_services();
  void display_startup_info();
  bool check_unity_endpoint_status();
  std::string process_message(const std::string & message);

  // Robot registry and monitoring
  struct RegisteredRobot
  {
    std::string robot_id;
    std::string name;
    std::string robot_type;
    std::string assigned_color;
    horus_interfaces::msg::RobotConfig config;
    std::vector<std::string> monitored_topics;
    std::map<std::string, double> topic_rates;
    std::map<std::string, bool> topic_active;
    rclcpp::Time last_seen;
    std::string health_status;
  };

  std::map<std::string, RegisteredRobot> registered_robots_;
  rclcpp::TimerBase::SharedPtr monitoring_timer_;

  // Robot management methods
  std::string generate_robot_id(const std::string & robot_name);
  std::string assign_robot_color(const std::string & robot_id);
  bool validate_robot_config(
    const horus_interfaces::msg::RobotConfig & config,
    std::vector<std::string> & errors);
  void monitor_robot_topics();
  void check_topic_activity(
    const std::string & robot_id,
    const std::string & topic);
  bool is_topic_active(const std::string & topic);
};

}  // namespace horus_backend
