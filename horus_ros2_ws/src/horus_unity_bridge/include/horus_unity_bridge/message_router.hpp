// SPDX-FileCopyrightText: 2025 RICE Lab, University of Genoa
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "protocol_handler.hpp"
#include "horus_unity_bridge/topic_manager.hpp"
#include "horus_unity_bridge/service_manager.hpp"

#ifdef ENABLE_WEBRTC
#include "horus_unity_bridge/webrtc_manager.hpp"
#endif

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <functional>
#include <unordered_map>
#include <mutex>
#include <vector>
#include <deque>
#include <thread>
#include <atomic>
#include <condition_variable>
#include <variant>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/image_encodings.hpp>

namespace horus_unity_bridge
{

/**
 * @brief Central message router for Unity-ROS2 communication
 * 
 * Single-node architecture that manages all publishers, subscribers, and services.
 * This avoids the overhead of creating separate nodes for each topic.
 * 
 * Features:
 * - Centralized ROS2 node for all topics
 * - Dynamic topic registration
 * - Efficient message routing
 * - Type introspection for dynamic message handling
 * - QoS configuration per topic
 */
class MessageRouter : public rclcpp::Node
{
public:
  using SendCallback = std::function<bool(int client_fd, const std::string&, const std::vector<uint8_t>&)>;
  using BroadcastCallback = std::function<void(const std::string&, const std::vector<uint8_t>&)>;
  
  explicit MessageRouter();
  ~MessageRouter();
  
  /**
   * @brief Set callbacks for sending messages to Unity clients
   */
  void set_send_callback(SendCallback callback) {
    send_callback_ = std::move(callback);
  }
  
  void set_broadcast_callback(BroadcastCallback callback) {
    broadcast_callback_ = std::move(callback);
  }
  
  /**
   * @brief Route an incoming message from Unity
   * 
   * @param client_fd Client that sent the message
   * @param message Protocol message
   * @return true if message was handled successfully
   */
  bool route_message(int client_fd, const ProtocolMessage& message);
  
  /**
   * @brief Handle system commands from Unity
   */
  bool handle_system_command(int client_fd, const ProtocolMessage& message);
  
  /**
   * @brief Get topic manager
   */
  TopicManager& get_topic_manager() { return *topic_manager_; }
  
  /**
   * @brief Get service manager
   */
  ServiceManager& get_service_manager() { return *service_manager_; }
  
  /**
   * @brief Send handshake payload to a client
   */
  void send_handshake(int client_fd);
  
  /**
   * @brief Get routing statistics
   */
  struct Statistics {
    uint64_t messages_routed = 0;
    uint64_t messages_published = 0;
    uint64_t messages_received = 0;
    uint64_t system_commands_processed = 0;
    uint64_t routing_errors = 0;
  };
  
  const Statistics& get_statistics() const { return stats_; }

private:
  // Managers
  std::unique_ptr<TopicManager> topic_manager_;
  std::unique_ptr<ServiceManager> service_manager_;

#ifdef ENABLE_WEBRTC
  std::unique_ptr<WebRTCManager> webrtc_manager_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_camera_sub_;
  bool webrtc_enabled_param_ = false;
  std::string webrtc_camera_topic_;
  int webrtc_bitrate_kbps_ = 2000;
  int webrtc_framerate_ = 30;
  std::string webrtc_encoder_;
  std::vector<std::string> webrtc_stun_servers_;
  std::string webrtc_pipeline_;
  bool warned_unsupported_encoding_ = false;
#endif
  
  // Callbacks
  SendCallback send_callback_;
  BroadcastCallback broadcast_callback_;
  
  // Statistics
  Statistics stats_;
  
  // System command handlers
  void handle_subscribe_command(int client_fd, const std::string& params);
  void handle_publish_command(int client_fd, const std::string& params);
  void handle_ros_service_command(int client_fd, const std::string& params);
  void handle_unity_service_command(int client_fd, const std::string& params);
  void handle_request_command(int client_fd, const std::string& params);
  void handle_response_command(int client_fd, const std::string& params);
  
#ifdef ENABLE_WEBRTC
  // WebRTC handlers
  void handle_webrtc_offer(int client_fd, const std::string& params);
  void handle_webrtc_answer(int client_fd, const std::string& params);
  void handle_webrtc_candidate(int client_fd, const std::string& params);
  void setup_camera_subscription(const std::string& topic);
  void handle_camera_frame(const sensor_msgs::msg::Image::SharedPtr& msg);
  void handle_compressed_camera_frame(const sensor_msgs::msg::CompressedImage::SharedPtr& msg);
  bool convert_to_rgb(const sensor_msgs::msg::Image& msg, std::vector<uint8_t>& output);
  bool decompress_to_rgb(const sensor_msgs::msg::CompressedImage& msg, std::vector<uint8_t>& output, int& width, int& height);

  // Async frame processing
  void process_frames();
  
  using FrameVariant = std::variant<sensor_msgs::msg::Image::SharedPtr, sensor_msgs::msg::CompressedImage::SharedPtr>;
  std::deque<FrameVariant> frame_queue_;
  std::mutex frame_queue_mutex_;
  std::condition_variable frame_queue_cv_;
  std::thread worker_thread_;
  std::atomic<bool> processing_running_{false};
  size_t max_queue_size_ = 2;
#endif

  void handle_topic_list_command(int client_fd);
  void handle_handshake_command(int client_fd);
  
  // Service request/response tracking
  uint32_t pending_service_request_id_ = 0;
  uint32_t pending_service_response_id_ = 0;
  // Map service request IDs to client FDs for routing responses
  std::unordered_map<uint32_t, int> service_response_client_;
  std::mutex service_response_mutex_;
  
  // Utility
  void send_error_to_client(int client_fd, const std::string& error_message);
  void send_log_to_client(int client_fd, const std::string& level, const std::string& message);
  
  bool parse_json_params(const std::string& json_str, 
                        std::unordered_map<std::string, std::string>& params);
};

} // namespace horus_unity_bridge
