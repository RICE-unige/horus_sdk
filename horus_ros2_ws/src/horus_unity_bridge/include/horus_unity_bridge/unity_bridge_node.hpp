// SPDX-FileCopyrightText: 2025 RICE Lab, University of Genoa
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "connection_manager.hpp"
#include "message_router.hpp"

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <atomic>

namespace horus_unity_bridge
{

/**
 * @brief Main HORUS Unity Bridge node
 * 
 * Integrates connection management and message routing for high-performance
 * Unity-ROS2 communication.
 * 
 * Key improvements over ROS-TCP-Endpoint:
 * - Single-node architecture (vs one node per topic)
 * - Epoll-based async I/O (vs Python threading)
 * - Zero-copy optimizations
 * - Configurable performance tuning
 * - Lower latency (~10x improvement expected)
 * - Higher throughput (~5x improvement expected)
 */
class UnityBridgeNode
{
public:
  UnityBridgeNode();
  ~UnityBridgeNode();
  
  /**
   * @brief Initialize and start the bridge
   */
  bool start();
  
  /**
   * @brief Stop the bridge and cleanup
   */
  void stop();
  
  /**
   * @brief Run the bridge (blocking until stopped)
   */
  void run();
  
  /**
   * @brief Check if bridge is running
   */
  bool is_running() const { return running_.load(); }
  
  /**
   * @brief Print comprehensive statistics
   */
  void print_statistics() const;

private:
  // Core components
  std::shared_ptr<MessageRouter> router_;
  std::unique_ptr<ConnectionManager> connection_manager_;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  
  // Configuration
  ConnectionManager::Config conn_config_;
  
  // State
  std::atomic<bool> running_;
  std::thread spin_thread_;
  
  // Private methods
  void load_parameters();
  void setup_callbacks();
  void handle_message_from_unity(int client_fd, const ProtocolMessage& message);
  void handle_client_connected(int client_fd, const std::string& ip, uint16_t port);
  void handle_client_disconnected(int client_fd, const std::string& ip, uint16_t port);
  
  // Statistics timer
  rclcpp::TimerBase::SharedPtr stats_timer_;
  void stats_timer_callback();
};

} // namespace horus_unity_bridge
