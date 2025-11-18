// SPDX-FileCopyrightText: 2025 RICE Lab, University of Genoa
// SPDX-License-Identifier: Apache-2.0

#include "horus_unity_bridge/unity_bridge_node.hpp"
#include <rclcpp/rclcpp.hpp>
#include <signal.h>
#include <iostream>

static horus_unity_bridge::UnityBridgeNode* g_node = nullptr;

void signal_handler(int signum)
{
  std::cout << "\nReceived signal " << signum << ", shutting down..." << std::endl;
  
  if (g_node) {
    g_node->stop();
  }
  
  rclcpp::shutdown();
}

int main(int argc, char** argv)
{
  // Initialize ROS
  rclcpp::init(argc, argv);
  
  // Setup signal handlers
  signal(SIGINT, signal_handler);
  signal(SIGTERM, signal_handler);
  
  try {
    // Create and start the Unity bridge node
    horus_unity_bridge::UnityBridgeNode node;
    g_node = &node;
    
    if (!node.start()) {
      RCLCPP_FATAL(rclcpp::get_logger("main"), "Failed to start Unity bridge");
      return 1;
    }
    
    std::cout << "\n========================================" << std::endl;
    std::cout << "  HORUS Unity Bridge" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "Bridge is running on port 10000" << std::endl;
    std::cout << "Press Ctrl+C to stop" << std::endl;
    std::cout << "========================================\n" << std::endl;
    
    // Run until shutdown
    node.run();
    
    std::cout << "\nShutting down gracefully..." << std::endl;
    node.stop();
    
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("main"), "Exception: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  
  rclcpp::shutdown();
  std::cout << "Unity bridge terminated successfully" << std::endl;
  
  return 0;
}
