// SPDX-FileCopyrightText: 2025 RICE Lab, University of Genoa
// SPDX-License-Identifier: Apache-2.0

#include "horus_unity_bridge/topic_manager.hpp"
#include <rclcpp/serialization.hpp>
#include <rosidl_typesupport_cpp/message_type_support.hpp>
#include <dlfcn.h>
#include <algorithm>

namespace horus_unity_bridge
{

TopicManager::TopicManager(rclcpp::Node* node)
  : node_(node)
{
  stats_ = TopicStatistics{};
}

bool TopicManager::register_publisher(const std::string& topic,
                                     const std::string& message_type,
                                     const rclcpp::QoS& qos)
{
  std::lock_guard<std::mutex> lock(publishers_mutex_);
  
  // Check if already registered
  if (publishers_.find(topic) != publishers_.end()) {
    return true;  // Already exists
  }
  
  try {
    // Normalize message type
    std::string normalized_type = normalize_message_type(message_type);
    
    // Create generic publisher
    auto publisher = node_->create_generic_publisher(topic, normalized_type, qos);
    
    // Store publisher info
    GenericPublisherInfo info;
    info.publisher = publisher;
    info.message_type = normalized_type;
    info.type_support = nullptr;  // Not needed for publishing
    
    publishers_[topic] = info;
    stats_.active_publishers++;
    
    node_->get_logger();
    RCLCPP_INFO(node_->get_logger(), "Registered publisher: %s [%s]", 
                topic.c_str(), normalized_type.c_str());
    
    return true;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to register publisher %s: %s",
                 topic.c_str(), e.what());
    return false;
  }
}

bool TopicManager::register_subscriber(const std::string& topic,
                                       const std::string& message_type,
                                       MessageCallback callback,
                                       const rclcpp::QoS& qos)
{
  std::lock_guard<std::mutex> lock(subscribers_mutex_);
  
  // Check if already registered
  if (subscribers_.find(topic) != subscribers_.end()) {
    return true;  // Already exists
  }
  
  try {
    // Normalize message type
    std::string normalized_type = normalize_message_type(message_type);
    
    // Create generic subscription with callback
    auto sub = node_->create_generic_subscription(
      topic,
      normalized_type,
      qos,
      [this, topic, callback](std::shared_ptr<rclcpp::SerializedMessage> msg) {
        // Convert serialized message to vector
        const auto& rcl_msg = msg->get_rcl_serialized_message();
        std::vector<uint8_t> data(rcl_msg.buffer, 
                                  rcl_msg.buffer + rcl_msg.buffer_length);
        
        // Call the callback
        callback(topic, data);
        
        stats_.messages_received++;
      }
    );
    
    // Store subscriber info
    GenericSubscriberInfo info;
    info.subscription = sub;
    info.message_type = normalized_type;
    info.callback = callback;
    info.type_support = nullptr;  // Not needed for subscription
    
    subscribers_[topic] = info;
    stats_.active_subscribers++;
    
    RCLCPP_INFO(node_->get_logger(), "Registered subscriber: %s [%s]",
                topic.c_str(), normalized_type.c_str());
    
    return true;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to register subscriber %s: %s",
                 topic.c_str(), e.what());
    return false;
  }
}

bool TopicManager::publish_message(const std::string& topic,
                                   const std::vector<uint8_t>& serialized_msg)
{
  std::lock_guard<std::mutex> lock(publishers_mutex_);
  
  auto it = publishers_.find(topic);
  if (it == publishers_.end()) {
    RCLCPP_WARN(node_->get_logger(), "Attempted to publish to unregistered topic: %s", 
                topic.c_str());
    stats_.publish_errors++;
    return false;
  }
  
  try {
    // Create ROS serialized message
    rclcpp::SerializedMessage msg(serialized_msg.size());
    auto& rcl_msg = msg.get_rcl_serialized_message();
    
    // Copy data
    std::memcpy(rcl_msg.buffer, serialized_msg.data(), serialized_msg.size());
    rcl_msg.buffer_length = serialized_msg.size();
    
    // Publish
    it->second.publisher->publish(msg);
    stats_.messages_published++;
    
    return true;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to publish message on %s: %s",
                 topic.c_str(), e.what());
    stats_.publish_errors++;
    return false;
  }
}

bool TopicManager::unregister_publisher(const std::string& topic)
{
  std::lock_guard<std::mutex> lock(publishers_mutex_);
  
  auto it = publishers_.find(topic);
  if (it == publishers_.end()) {
    return false;
  }
  
  publishers_.erase(it);
  stats_.active_publishers--;
  
  RCLCPP_INFO(node_->get_logger(), "Unregistered publisher: %s", topic.c_str());
  return true;
}

bool TopicManager::unregister_subscriber(const std::string& topic)
{
  std::lock_guard<std::mutex> lock(subscribers_mutex_);
  
  auto it = subscribers_.find(topic);
  if (it == subscribers_.end()) {
    return false;
  }
  
  subscribers_.erase(it);
  stats_.active_subscribers--;
  
  RCLCPP_INFO(node_->get_logger(), "Unregistered subscriber: %s", topic.c_str());
  return true;
}

bool TopicManager::has_publisher(const std::string& topic) const
{
  std::lock_guard<std::mutex> lock(publishers_mutex_);
  return publishers_.find(topic) != publishers_.end();
}

bool TopicManager::has_subscriber(const std::string& topic) const
{
  std::lock_guard<std::mutex> lock(subscribers_mutex_);
  return subscribers_.find(topic) != subscribers_.end();
}

std::vector<std::string> TopicManager::get_publisher_topics() const
{
  std::lock_guard<std::mutex> lock(publishers_mutex_);
  std::vector<std::string> topics;
  topics.reserve(publishers_.size());
  
  for (const auto& pair : publishers_) {
    topics.push_back(pair.first);
  }
  
  return topics;
}

std::vector<std::string> TopicManager::get_subscriber_topics() const
{
  std::lock_guard<std::mutex> lock(subscribers_mutex_);
  std::vector<std::string> topics;
  topics.reserve(subscribers_.size());
  
  for (const auto& pair : subscribers_) {
    topics.push_back(pair.first);
  }
  
  return topics;
}

std::string TopicManager::normalize_message_type(const std::string& message_type)
{
  // Convert from Unity format (package/MessageType) to ROS2 format (package/msg/MessageType)
  std::string normalized = message_type;
  
  // If already in correct format, return as-is
  if (normalized.find("/msg/") != std::string::npos || 
      normalized.find("/srv/") != std::string::npos) {
    return normalized;
  }
  
  // Find the slash position
  size_t slash_pos = normalized.find('/');
  if (slash_pos != std::string::npos) {
    // Insert "/msg/" or "/srv/" in the middle
    // Default to "/msg/" for topics
    std::string package = normalized.substr(0, slash_pos);
    std::string type_name = normalized.substr(slash_pos + 1);
    normalized = package + "/msg/" + type_name;
  }
  
  return normalized;
}

const rosidl_message_type_support_t* TopicManager::get_type_support(
    const std::string& message_type)
{
  // This function would dynamically load type support
  // For now, we rely on generic publishers/subscribers which don't need explicit type support
  return nullptr;
}

} // namespace horus_unity_bridge
