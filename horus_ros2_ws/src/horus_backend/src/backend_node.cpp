// SPDX-FileCopyrightText: 2025 RICE Lab, University of Genoa
// SPDX-License-Identifier: Apache-2.0

#include "horus_backend/backend_node.hpp"

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <functional>
#include <iostream>
#include <map>
#include <vector>

namespace horus_backend
{

BackendNode::BackendNode()
: Node("horus_backend_node")
{
  setup_parameters();
  setup_publishers();
  setup_subscribers();
  setup_services();
}

BackendNode::~BackendNode() {shutdown();}

void BackendNode::initialize()
{
  // Initialize plugin manager
  plugin_manager_ = std::make_unique<PluginManager>();
  plugin_manager_->load_plugins();

  // Initialize TCP server
  tcp_server_ = std::make_unique<TcpServer>(tcp_port_);
  tcp_server_->set_message_callback(
    [this](const std::string & message) {return process_message(message);});

  // Start TCP server
  tcp_server_->start();

  display_startup_info();
}

void BackendNode::shutdown()
{
  RCLCPP_INFO(get_logger(), "Shutting down HORUS Backend...");

  if (tcp_server_) {
    tcp_server_->stop();
  }

  RCLCPP_INFO(get_logger(), "HORUS Backend shutdown complete");
}

void BackendNode::setup_parameters()
{
  // Declare parameters with defaults
  declare_parameter<int>("tcp_port", 8080);
  declare_parameter<int>("unity_tcp_port", 10000);
  declare_parameter<std::string>("log_level", "info");

  // Get parameter values
  tcp_port_ = get_parameter("tcp_port").as_int();
  unity_tcp_port_ = get_parameter("unity_tcp_port").as_int();
  log_level_ = get_parameter("log_level").as_string();
}

void BackendNode::setup_publishers()
{
  command_publisher_ = create_publisher<horus_interfaces::msg::RobotCommand>(
    "horus/robot_commands", 10);

  status_publisher_ = create_publisher<horus_interfaces::msg::RobotStatus>(
    "horus/robot_status", 10);
}

void BackendNode::setup_subscribers()
{
  status_subscriber_ = create_subscription<horus_interfaces::msg::RobotStatus>(
    "horus/robot_status_feedback", 10,
    std::bind(&BackendNode::status_callback, this, std::placeholders::_1));
}

void BackendNode::setup_services()
{
  connect_service_ = create_service<horus_interfaces::srv::ConnectRobot>(
    "horus/connect_robot",
    std::bind(
      &BackendNode::connect_robot_callback, this,
      std::placeholders::_1, std::placeholders::_2));

  execute_service_ = create_service<horus_interfaces::srv::ExecuteCommand>(
    "horus/execute_command",
    std::bind(
      &BackendNode::execute_command_callback, this,
      std::placeholders::_1, std::placeholders::_2));

  register_service_ = create_service<horus_interfaces::srv::RegisterRobot>(
    "horus/register_robot",
    std::bind(
      &BackendNode::register_robot_callback, this,
      std::placeholders::_1, std::placeholders::_2));

  unregister_service_ = create_service<horus_interfaces::srv::UnregisterRobot>(
    "horus/unregister_robot",
    std::bind(
      &BackendNode::unregister_robot_callback, this,
      std::placeholders::_1, std::placeholders::_2));
}

void BackendNode::status_callback(
  const horus_interfaces::msg::RobotStatus::SharedPtr msg)
{
  RCLCPP_DEBUG(
    get_logger(), "Received status from robot: %s",
    msg->robot_id.c_str());
  // Forward status to connected SDK clients via TCP
  // This will be implemented when we add the TCP message processing
}

void BackendNode::connect_robot_callback(
  const std::shared_ptr<horus_interfaces::srv::ConnectRobot::Request> request,
  std::shared_ptr<horus_interfaces::srv::ConnectRobot::Response> response)
{
  RCLCPP_INFO(
    get_logger(), "Connect robot request: %s (type: %s)",
    request->robot_id.c_str(), request->robot_type.c_str());

  bool success =
    plugin_manager_->connect_robot(request->robot_id, request->robot_type);
  response->success = success;
  response->message =
    success ? "Robot connected successfully" : "Failed to connect robot";
}

void BackendNode::execute_command_callback(
  const std::shared_ptr<horus_interfaces::srv::ExecuteCommand::Request>
  request,
  std::shared_ptr<horus_interfaces::srv::ExecuteCommand::Response> response)
{
  RCLCPP_INFO(
    get_logger(), "Execute command for robot: %s, command: %s",
    request->robot_id.c_str(), request->command_type.c_str());

  // For now, just acknowledge the command
  response->success = true;
  response->result_data = "Command executed";
  response->message = "Command processed successfully";
}

void BackendNode::display_startup_info()
{
  RCLCPP_INFO(get_logger(), "========================================");
  RCLCPP_INFO(get_logger(), "    HORUS ROS2 Backend Node");
  RCLCPP_INFO(get_logger(), "========================================");
  RCLCPP_INFO(get_logger(), "Version: 0.1.0");
  RCLCPP_INFO(get_logger(), "TCP Port: %d", tcp_port_);
  RCLCPP_INFO(get_logger(), "Unity TCP Port: %d", unity_tcp_port_);
  RCLCPP_INFO(get_logger(), "Log Level: %s", log_level_.c_str());
  RCLCPP_INFO(
    get_logger(), "Available Plugins: %zu",
    plugin_manager_->get_available_plugins().size());

  // Check Unity endpoint status
  if (check_unity_endpoint_status()) {
    RCLCPP_INFO(
      get_logger(), "Unity TCP Endpoint: Available on port %d",
      unity_tcp_port_);
  } else {
    RCLCPP_WARN(
      get_logger(), "Unity TCP Endpoint: Not detected on port %d",
      unity_tcp_port_);
  }

  RCLCPP_INFO(get_logger(), "========================================");
  RCLCPP_INFO(get_logger(), "Backend ready for SDK connections");
}

bool BackendNode::check_unity_endpoint_status()
{
  // Check if Unity TCP endpoint is available by attempting a connection
  int sock = socket(AF_INET, SOCK_STREAM, 0);
  if (sock < 0) {
    return false;
  }

  struct sockaddr_in server_addr;
  server_addr.sin_family = AF_INET;
  server_addr.sin_port = htons(unity_tcp_port_);
  server_addr.sin_addr.s_addr = inet_addr("127.0.0.1");

  // Set socket to non-blocking for quick check
  int flags = fcntl(sock, F_GETFL, 0);
  fcntl(sock, F_SETFL, flags | O_NONBLOCK);

  bool is_available = false;
  int result =
    connect(sock, (struct sockaddr *)&server_addr, sizeof(server_addr));

  if (result == 0 || errno == EISCONN) {
    is_available = true;
  } else if (errno == EINPROGRESS) {
    // Connection in progress, wait briefly
    fd_set write_fds;
    FD_ZERO(&write_fds);
    FD_SET(sock, &write_fds);

    struct timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec = 100000;  // 100ms timeout

    if (select(sock + 1, NULL, &write_fds, NULL, &timeout) > 0) {
      int error = 0;
      socklen_t len = sizeof(error);
      if (getsockopt(sock, SOL_SOCKET, SO_ERROR, &error, &len) == 0 &&
        error == 0)
      {
        is_available = true;
      }
    }
  }

  close(sock);
  return is_available;
}

std::string BackendNode::process_message(const std::string & message)
{
  // Basic message processing - will be expanded later
  RCLCPP_DEBUG(get_logger(), "Received TCP message: %s", message.c_str());

  // For now, just echo back a simple response
  return "{\"status\": \"ok\", \"message\": \"Backend received: " + message +
         "\"}";
}

void BackendNode::register_robot_callback(
  const std::shared_ptr<horus_interfaces::srv::RegisterRobot::Request>
  request,
  std::shared_ptr<horus_interfaces::srv::RegisterRobot::Response> response)
{
  RCLCPP_INFO(
    get_logger(), "Robot registration request: %s (type: %s)",
    request->robot_config.name.c_str(),
    request->robot_config.robot_type.c_str());

  // Validate robot configuration
  std::vector<std::string> validation_errors;
  if (!validate_robot_config(request->robot_config, validation_errors)) {
    response->success = false;
    response->error_message = "Robot configuration validation failed";
    response->validation_errors = validation_errors;
    RCLCPP_ERROR(
      get_logger(), "Robot registration failed: %s",
      response->error_message.c_str());
    return;
  }

  // Generate unique robot ID
  std::string robot_id = generate_robot_id(request->robot_config.name);

  // Assign color for MR visualization
  std::string assigned_color = assign_robot_color(robot_id);

  // Create registered robot entry
  RegisteredRobot robot;
  robot.robot_id = robot_id;
  robot.name = request->robot_config.name;
  robot.robot_type = request->robot_config.robot_type;
  robot.assigned_color = assigned_color;
  robot.config = request->robot_config;
  robot.last_seen = now();
  robot.health_status = "unknown";

  // Extract monitored topics from sensors and control topics
  for (const auto & sensor : request->robot_config.sensors) {
    robot.monitored_topics.push_back(sensor.topic);
    robot.topic_active[sensor.topic] = false;
    robot.topic_rates[sensor.topic] = 0.0;
  }

  for (const auto & topic : request->robot_config.control_topics) {
    robot.monitored_topics.push_back(topic);
    robot.topic_active[topic] = false;
    robot.topic_rates[topic] = 0.0;
  }

  for (const auto & topic : request->robot_config.status_topics) {
    robot.monitored_topics.push_back(topic);
    robot.topic_active[topic] = false;
    robot.topic_rates[topic] = 0.0;
  }

  // Register robot
  registered_robots_[robot_id] = robot;

  // Start monitoring timer if this is the first robot
  if (registered_robots_.size() == 1 && !monitoring_timer_) {
    monitoring_timer_ =
      create_wall_timer(
      std::chrono::seconds(2),
      std::bind(&BackendNode::monitor_robot_topics, this));
  }

  // Prepare response
  response->success = true;
  response->robot_id = robot_id;
  response->assigned_color = assigned_color;
  response->error_message = "";

  RCLCPP_INFO(
    get_logger(),
    "Robot registered successfully: %s -> %s (color: %s)",
    request->robot_config.name.c_str(), robot_id.c_str(),
    assigned_color.c_str());
}

void BackendNode::unregister_robot_callback(
  const std::shared_ptr<horus_interfaces::srv::UnregisterRobot::Request>
  request,
  std::shared_ptr<horus_interfaces::srv::UnregisterRobot::Response>
  response)
{
  RCLCPP_INFO(
    get_logger(), "Robot unregistration request: %s",
    request->robot_id.c_str());

  auto it = registered_robots_.find(request->robot_id);
  if (it != registered_robots_.end()) {
    registered_robots_.erase(it);
    response->success = true;
    response->error_message = "";

    // Stop monitoring if no more robots
    if (registered_robots_.empty() && monitoring_timer_) {
      monitoring_timer_.reset();
    }

    RCLCPP_INFO(
      get_logger(), "Robot unregistered successfully: %s",
      request->robot_id.c_str());
  } else {
    response->success = false;
    response->error_message = "Robot not found";
    RCLCPP_ERROR(
      get_logger(), "Robot unregistration failed: %s not found",
      request->robot_id.c_str());
  }
}

std::string BackendNode::generate_robot_id(const std::string & robot_name)
{
  // Create unique ID based on name and timestamp
  auto now_time = std::chrono::system_clock::now();
  auto timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
    now_time.time_since_epoch())
    .count();
  return robot_name + "_" + std::to_string(timestamp);
}

std::string BackendNode::assign_robot_color(const std::string & robot_id)
{
  // Simple color assignment based on hash of robot_id
  std::hash<std::string> hasher;
  size_t hash = hasher(robot_id);

  // Predefined bright colors for good MR visibility
  std::vector<std::string> colors = {"#FF0000", "#00FF00", "#0000FF", "#FF7F00",
    "#FF00FF", "#00FFFF", "#FFFF00", "#FF007F",
    "#7F00FF", "#00FF7F"};

  return colors[hash % colors.size()];
}

bool BackendNode::validate_robot_config(
  const horus_interfaces::msg::RobotConfig & config,
  std::vector<std::string> & errors)
{
  bool valid = true;

  // Check basic fields
  if (config.name.empty()) {
    errors.push_back("Robot name cannot be empty");
    valid = false;
  }

  if (config.robot_type.empty()) {
    errors.push_back("Robot type cannot be empty");
    valid = false;
  }

  // Check if robot name is already registered
  for (const auto & pair : registered_robots_) {
    if (pair.second.name == config.name) {
      errors.push_back("Robot name already registered: " + config.name);
      valid = false;
      break;
    }
  }

  // Validate sensors
  for (const auto & sensor : config.sensors) {
    if (sensor.name.empty()) {
      errors.push_back("Sensor name cannot be empty");
      valid = false;
    }
    if (sensor.topic.empty()) {
      errors.push_back(
        "Sensor topic cannot be empty for sensor: " +
        sensor.name);
      valid = false;
    }
  }

  return valid;
}

void BackendNode::monitor_robot_topics()
{
  for (auto & pair : registered_robots_) {
    auto & robot = pair.second;
    bool any_topic_active = false;

    for (const auto & topic : robot.monitored_topics) {
      bool active = is_topic_active(topic);
      robot.topic_active[topic] = active;

      if (active) {
        any_topic_active = true;
      }
    }

    // Update robot health based on topic activity
    if (any_topic_active) {
      robot.health_status = "healthy";
    } else {
      robot.health_status = "warning";
    }

    robot.last_seen = now();

    RCLCPP_DEBUG(
      get_logger(), "Robot %s health: %s", robot.robot_id.c_str(),
      robot.health_status.c_str());
  }
}

bool BackendNode::is_topic_active(const std::string & topic)
{
  // Check if topic has active publishers
  auto topic_info = get_publishers_info_by_topic(topic);
  return !topic_info.empty();
}

}  // namespace horus_backend
