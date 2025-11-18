// SPDX-FileCopyrightText: 2025 RICE Lab, University of Genoa
// SPDX-License-Identifier: Apache-2.0

#include "horus_unity_bridge/unity_bridge_node.hpp"
#include <iostream>
#include <chrono>

namespace horus_unity_bridge
{

UnityBridgeNode::UnityBridgeNode()
  : running_(false)
{
  // Initialize ROS if not already done
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  
  // Load parameters
  load_parameters();
  
  // Create router (ROS node)
  router_ = std::make_shared<MessageRouter>();
  
  // Create connection manager
  connection_manager_ = std::make_unique<ConnectionManager>(conn_config_);
  
  // Setup callbacks
  setup_callbacks();
  
  // Create executor
  int num_threads = 4;  // TODO: Make configurable
  rclcpp::ExecutorOptions options;
  executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(
    options, num_threads, false
  );
  executor_->add_node(router_);
  
  RCLCPP_INFO(router_->get_logger(), "Unity Bridge Node initialized");
}

UnityBridgeNode::~UnityBridgeNode()
{
  stop();
}

void UnityBridgeNode::load_parameters()
{
  // Load configuration from ROS parameters or use defaults
  conn_config_.bind_address = "0.0.0.0";
  conn_config_.port = 10000;
  conn_config_.max_connections = 10;
  conn_config_.socket_buffer_size = 65536;
  conn_config_.tcp_nodelay = true;
  conn_config_.connection_timeout_ms = 5000;
  
  // TODO: Load from ROS parameters when node is created
}

void UnityBridgeNode::setup_callbacks()
{
  // Message from Unity -> Router
  connection_manager_->set_message_callback(
    [this](int fd, const ProtocolMessage& msg) {
      handle_message_from_unity(fd, msg);
    }
  );
  
  // Router wants to send to Unity -> Connection Manager  
  router_->set_send_callback(
    [this](int fd, const std::string& dest, const std::vector<uint8_t>& payload) {
      return connection_manager_->send_to_client(fd, dest, payload);
    }
  );
  
  router_->set_broadcast_callback(
    [this](const std::string& dest, const std::vector<uint8_t>& payload) {
      connection_manager_->broadcast_message(dest, payload);
    }
  );
  
  // Connection events
  connection_manager_->set_connection_callback(
    [this](int fd, const std::string& ip, uint16_t port) {
      handle_client_connected(fd, ip, port);
    }
  );
  
  connection_manager_->set_disconnection_callback(
    [this](int fd, const std::string& ip, uint16_t port) {
      handle_client_disconnected(fd, ip, port);
    }
  );
}

bool UnityBridgeNode::start()
{
  if (running_.load()) {
    return true;
  }
  
  // Start connection manager
  if (!connection_manager_->start()) {
    RCLCPP_ERROR(router_->get_logger(), "Failed to start connection manager");
    return false;
  }
  
  running_ = true;
  
  // Start ROS executor in separate thread
  spin_thread_ = std::thread([this]() {
    RCLCPP_INFO(router_->get_logger(), "ROS executor thread started");
    executor_->spin();
    RCLCPP_INFO(router_->get_logger(), "ROS executor thread stopped");
  });
  
  // Create statistics timer
  stats_timer_ = router_->create_wall_timer(
    std::chrono::seconds(30),
    [this]() { stats_timer_callback(); }
  );
  
  // Create service response processing timer (1ms for low latency)
  auto service_timer = router_->create_wall_timer(
    std::chrono::milliseconds(1),
    [this]() { router_->get_service_manager().process_pending_responses(); }
  );
  
  RCLCPP_INFO(router_->get_logger(), "Unity Bridge started successfully");
  
  return true;
}

void UnityBridgeNode::stop()
{
  if (!running_.load()) {
    return;
  }
  
  RCLCPP_INFO(router_->get_logger(), "Stopping Unity Bridge...");
  
  running_ = false;
  
  // Stop statistics timer
  if (stats_timer_) {
    stats_timer_->cancel();
  }
  
  // Stop connection manager
  connection_manager_->stop();
  
  // Stop ROS executor
  if (executor_) {
    executor_->cancel();
  }
  
  // Wait for spin thread
  if (spin_thread_.joinable()) {
    spin_thread_.join();
  }
  
  RCLCPP_INFO(router_->get_logger(), "Unity Bridge stopped");
}

void UnityBridgeNode::run()
{
  // Wait for shutdown signal
  while (running_.load() && rclcpp::ok()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

void UnityBridgeNode::handle_message_from_unity(int client_fd, const ProtocolMessage& message)
{
  // Route message through the router
  router_->route_message(client_fd, message);
}

void UnityBridgeNode::handle_client_connected(int client_fd, const std::string& ip, uint16_t port)
{
  RCLCPP_INFO(router_->get_logger(), "Unity client connected: %s:%u (fd=%d)",
              ip.c_str(), port, client_fd);
  router_->send_handshake(client_fd);
}

void UnityBridgeNode::handle_client_disconnected(int client_fd, const std::string& ip, uint16_t port)
{
  RCLCPP_INFO(router_->get_logger(), "Unity client disconnected: %s:%u (fd=%d)",
              ip.c_str(), port, client_fd);
}

void UnityBridgeNode::stats_timer_callback()
{
  print_statistics();
}

void UnityBridgeNode::print_statistics() const
{
  auto conn_stats = connection_manager_->get_statistics();
  auto router_stats = router_->get_statistics();
  auto topic_stats = router_->get_topic_manager().get_statistics();
  auto service_stats = router_->get_service_manager().get_statistics();
  
  RCLCPP_INFO(router_->get_logger(), "=== Unity Bridge Statistics ===");
  RCLCPP_INFO(router_->get_logger(), "Connections: %lu active, %lu total",
              conn_stats.active_connections, conn_stats.total_connections);
  RCLCPP_INFO(router_->get_logger(), "Messages: %lu sent, %lu received",
              conn_stats.messages_sent, conn_stats.messages_received);
  RCLCPP_INFO(router_->get_logger(), "Bytes: %lu sent, %lu received",
              conn_stats.bytes_sent, conn_stats.bytes_received);
  RCLCPP_INFO(router_->get_logger(), "Topics: %lu publishers, %lu subscribers",
              topic_stats.active_publishers, topic_stats.active_subscribers);
  RCLCPP_INFO(router_->get_logger(), "Router: %lu routed, %lu published, %lu commands",
              router_stats.messages_routed, router_stats.messages_published,
              router_stats.system_commands_processed);
  RCLCPP_INFO(router_->get_logger(), "Services: %lu ROS calls, %lu Unity calls, %lu responses, %lu timeouts, %lu errors",
              service_stats.ros_services_called, service_stats.unity_services_called,
              service_stats.responses_sent, service_stats.timeouts, service_stats.errors);
  
  if (conn_stats.connection_errors > 0 || router_stats.routing_errors > 0) {
    RCLCPP_WARN(router_->get_logger(), "Errors: %lu connection, %lu routing",
                conn_stats.connection_errors, router_stats.routing_errors);
  }
}

} // namespace horus_unity_bridge
