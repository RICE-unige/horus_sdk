// SPDX-FileCopyrightText: 2025 RICE Lab, University of Genoa
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rosidl_typesupport_cpp/message_type_support.hpp>
#include <rosidl_typesupport_introspection_cpp/message_introspection.hpp>

#include <memory>
#include <string>
#include <vector>
#include <unordered_map>
#include <functional>
#include <mutex>

namespace horus_unity_bridge
{

/**
 * @brief Manages dynamic ROS2 publishers and subscribers
 * 
 * Features:
 * - Dynamic type loading and introspection
 * - Efficient memory pooling for messages
 * - Per-topic QoS configuration
 * - Zero-copy message passing where possible
 * - Thread-safe topic registration/unregistration
 */
class TopicManager
{
public:
  using MessageCallback = std::function<void(const std::string& topic, 
                                             const std::vector<uint8_t>& data)>;
  
  explicit TopicManager(rclcpp::Node* node);
  ~TopicManager() = default;
  
  /**
   * @brief Register a publisher for Unity to publish ROS messages
   * 
   * @param topic Topic name
   * @param message_type Message type (e.g., "std_msgs/String")
   * @param qos QoS profile
   * @return true if successful
   */
  bool register_publisher(const std::string& topic, 
                         const std::string& message_type,
                         const rclcpp::QoS& qos = rclcpp::QoS(10));
  
  /**
   * @brief Register a subscriber to forward ROS messages to Unity
   * 
   * @param topic Topic name
   * @param message_type Message type
   * @param callback Callback when message received
   * @param qos QoS profile
   * @return true if successful
   */
  bool register_subscriber(const std::string& topic,
                          const std::string& message_type,
                          MessageCallback callback,
                          const rclcpp::QoS& qos = rclcpp::QoS(10));
  
  /**
   * @brief Publish a message from Unity to ROS
   * 
   * @param topic Topic name
   * @param serialized_msg Serialized ROS message
   * @return true if successful
   */
  bool publish_message(const std::string& topic, 
                      const std::vector<uint8_t>& serialized_msg);
  
  /**
   * @brief Unregister a publisher
   */
  bool unregister_publisher(const std::string& topic);
  
  /**
   * @brief Unregister a subscriber
   */
  bool unregister_subscriber(const std::string& topic);
  
  /**
   * @brief Check if topic is registered as publisher
   */
  bool has_publisher(const std::string& topic) const;
  
  /**
   * @brief Check if topic is registered as subscriber
   */
  bool has_subscriber(const std::string& topic) const;
  
  /**
   * @brief Get list of registered publishers
   */
  std::vector<std::string> get_publisher_topics() const;
  
  /**
   * @brief Get list of registered subscribers
   */
  std::vector<std::string> get_subscriber_topics() const;
  
  /**
   * @brief Get topic statistics
   */
  struct TopicStatistics {
    uint64_t messages_published = 0;
    uint64_t messages_received = 0;
    uint64_t publish_errors = 0;
    uint64_t active_publishers = 0;
    uint64_t active_subscribers = 0;
  };
  
  const TopicStatistics& get_statistics() const { return stats_; }

private:
  rclcpp::Node* node_;
  
  // Generic publisher/subscriber holders
  struct GenericPublisherInfo {
    std::shared_ptr<rclcpp::GenericPublisher> publisher;
    std::string message_type;
    const rosidl_message_type_support_t* type_support;
  };
  
  struct GenericSubscriberInfo {
    std::shared_ptr<rclcpp::GenericSubscription> subscription;
    std::string message_type;
    MessageCallback callback;
    const rosidl_message_type_support_t* type_support;
  };
  
  std::unordered_map<std::string, GenericPublisherInfo> publishers_;
  std::unordered_map<std::string, GenericSubscriberInfo> subscribers_;
  
  mutable std::mutex publishers_mutex_;
  mutable std::mutex subscribers_mutex_;
  
  TopicStatistics stats_;
  
  // Helper methods
  const rosidl_message_type_support_t* get_type_support(const std::string& message_type);
  std::string normalize_message_type(const std::string& message_type);
  
  void generic_subscription_callback(
    const std::string& topic,
    const std::shared_ptr<rclcpp::SerializedMessage>& serialized_msg
  );
};

} // namespace horus_unity_bridge
