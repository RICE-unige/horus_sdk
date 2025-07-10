// SPDX-FileCopyrightText: 2025 RICE Lab, University of Genoa
// SPDX-License-Identifier: Apache-2.0

#include <signal.h>

#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "horus_backend/backend_node.hpp"

std::shared_ptr<horus_backend::BackendNode> g_node = nullptr;

void signal_handler(int signum)
{
  if (g_node) {
    RCLCPP_INFO(
      g_node->get_logger(), "Received signal %d, shutting down...",
      signum);
    g_node->shutdown();
    rclcpp::shutdown();
  }
  exit(signum);
}

int main(int argc, char ** argv)
{
  // Initialize ROS2
  rclcpp::init(argc, argv);

  // Setup signal handling
  signal(SIGINT, signal_handler);
  signal(SIGTERM, signal_handler);

  try {
    // Create and initialize backend node
    g_node = std::make_shared<horus_backend::BackendNode>();
    g_node->initialize();

    RCLCPP_INFO(
      g_node->get_logger(),
      "HORUS Backend Node started successfully");

    // Spin the node
    rclcpp::spin(g_node);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("main"), "Exception: %s", e.what());
    return 1;
  }

  // Cleanup
  if (g_node) {
    g_node->shutdown();
  }

  rclcpp::shutdown();
  return 0;
}
