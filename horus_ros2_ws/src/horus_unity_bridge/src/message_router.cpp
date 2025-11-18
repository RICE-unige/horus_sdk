// SPDX-FileCopyrightText: 2025 RICE Lab, University of Genoa
// SPDX-License-Identifier: Apache-2.0

#include "horus_unity_bridge/message_router.hpp"
#include <sstream>
#include <nlohmann/json.hpp>

namespace horus_unity_bridge
{

MessageRouter::MessageRouter()
  : Node("horus_unity_bridge")
{
  topic_manager_ = std::make_unique<TopicManager>(this);
  service_manager_ = std::make_unique<ServiceManager>(this);
  
  // Setup service manager callback for sending responses to Unity
  service_manager_->set_response_callback(
    [this](uint32_t srv_id, const std::vector<uint8_t>& response) {
      // Send service response back to the requesting Unity client
      int target_fd = -1;
      {
        std::lock_guard<std::mutex> lock(service_response_mutex_);
        auto it = service_response_client_.find(srv_id);
        if (it != service_response_client_.end()) {
          target_fd = it->second;
          service_response_client_.erase(it);
        }
      }
      if (send_callback_ && target_fd != -1) {
        // First send __response command with srv_id
        nlohmann::json response_cmd;
        response_cmd["srv_id"] = srv_id;
        std::string json_str = response_cmd.dump();
        std::vector<uint8_t> cmd_data(json_str.begin(), json_str.end());
        send_callback_(target_fd, "__response", cmd_data);
        
        // Then send the serialized response payload
        send_callback_(target_fd, "__response_payload", response);
      }
    }
  );
  
  stats_ = Statistics{};
  
  RCLCPP_INFO(get_logger(), "Message router initialized");
}

bool MessageRouter::route_message(int client_fd, const ProtocolMessage& message)
{
  stats_.messages_routed++;
  
  // Ignore keepalive messages
  if (message.is_keepalive()) {
    return true;
  }
  
  // Handle system commands
  if (message.is_system_command()) {
    return handle_system_command(client_fd, message);
  }
  
  // If a service request was announced, treat this payload as the request body
  if (pending_service_request_id_ != 0) {
    uint32_t srv_id = pending_service_request_id_;
    pending_service_request_id_ = 0;
    bool ok = service_manager_->call_ros_service_async(srv_id, message.destination, message.payload);
    if (!ok) {
      stats_.routing_errors++;
      return false;
    }
    return true;
  }
  
  // If a service response was announced, treat this payload as Unity's response
  if (pending_service_response_id_ != 0) {
    uint32_t srv_id = pending_service_response_id_;
    pending_service_response_id_ = 0;
    service_manager_->handle_unity_service_response(srv_id, message.payload);
    return true;
  }
  
  // Regular message - publish to ROS
  bool success = topic_manager_->publish_message(message.destination, message.payload);
  
  if (success) {
    stats_.messages_published++;
  } else {
    stats_.routing_errors++;
  }
  
  return success;
}

bool MessageRouter::handle_system_command(int client_fd, const ProtocolMessage& message)
{
  stats_.system_commands_processed++;
  
  std::string command = message.destination;
  std::string params_json(message.payload.begin(), message.payload.end());
  
  RCLCPP_DEBUG(get_logger(), "System command: %s", command.c_str());
  
  try {
    if (command == "__subscribe") {
      handle_subscribe_command(client_fd, params_json);
    } else if (command == "__publish") {
      handle_publish_command(client_fd, params_json);
    } else if (command == "__topic_list") {
      handle_topic_list_command(client_fd);
    } else if (command == "__handshake") {
      handle_handshake_command(client_fd);
    } else if (command == "__ros_service") {
      handle_ros_service_command(client_fd, params_json);
    } else if (command == "__unity_service") {
      handle_unity_service_command(client_fd, params_json);
    } else if (command == "__request") {
      handle_request_command(client_fd, params_json);
    } else if (command == "__response") {
      handle_response_command(client_fd, params_json);
    } else {
      RCLCPP_WARN(get_logger(), "Unknown system command: %s", command.c_str());
      send_error_to_client(client_fd, "Unknown system command: " + command);
      return false;
    }
    
    return true;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "Error handling system command %s: %s", 
                 command.c_str(), e.what());
    send_error_to_client(client_fd, std::string("Error: ") + e.what());
    stats_.routing_errors++;
    return false;
  }
}

void MessageRouter::handle_subscribe_command(int client_fd, const std::string& params)
{
  // Parse JSON parameters
  std::unordered_map<std::string, std::string> param_map;
  if (!parse_json_params(params, param_map)) {
    send_error_to_client(client_fd, "Failed to parse subscribe parameters");
    return;
  }
  
  std::string topic = param_map["topic"];
  std::string message_name = param_map["message_name"];
  
  if (topic.empty() || message_name.empty()) {
    send_error_to_client(client_fd, "Missing required parameters: topic, message_name");
    return;
  }
  
  // Register subscriber to forward ROS messages to Unity
  bool success = topic_manager_->register_subscriber(
    topic,
    message_name,
    [this, client_fd](const std::string& topic, const std::vector<uint8_t>& data) {
      // Forward to Unity client
      if (send_callback_) {
        send_callback_(client_fd, topic, data);
      }
    }
  );
  
  if (success) {
    RCLCPP_INFO(get_logger(), "Registered Unity subscriber: %s [%s]", 
                topic.c_str(), message_name.c_str());
    send_log_to_client(client_fd, "info", 
                      "RegisterSubscriber(" + topic + ", " + message_name + ") OK");
  } else {
    send_error_to_client(client_fd, "Failed to register subscriber: " + topic);
  }
}

void MessageRouter::handle_publish_command(int client_fd, const std::string& params)
{
  // Parse JSON parameters
  std::unordered_map<std::string, std::string> param_map;
  if (!parse_json_params(params, param_map)) {
    send_error_to_client(client_fd, "Failed to parse publish parameters");
    return;
  }
  
  std::string topic = param_map["topic"];
  std::string message_name = param_map["message_name"];
  
  if (topic.empty() || message_name.empty()) {
    send_error_to_client(client_fd, "Missing required parameters: topic, message_name");
    return;
  }
  
  // Register publisher so Unity can publish to ROS
  bool success = topic_manager_->register_publisher(topic, message_name);
  
  if (success) {
    RCLCPP_INFO(get_logger(), "Registered Unity publisher: %s [%s]", 
                topic.c_str(), message_name.c_str());
    send_log_to_client(client_fd, "info",
                      "RegisterPublisher(" + topic + ", " + message_name + ") OK");
  } else {
    send_error_to_client(client_fd, "Failed to register publisher: " + topic);
  }
}

void MessageRouter::handle_ros_service_command(int client_fd, const std::string& params)
{
  // Parse JSON parameters
  std::unordered_map<std::string, std::string> param_map;
  if (!parse_json_params(params, param_map)) {
    send_error_to_client(client_fd, "Failed to parse ROS service parameters");
    return;
  }
  
  std::string service_name = param_map["topic"];
  std::string service_type = param_map["message_name"];
  
  if (service_name.empty() || service_type.empty()) {
    send_error_to_client(client_fd, "Missing required parameters: topic, message_name");
    return;
  }
  
  // Register ROS service client (Unity will call this ROS service)
  bool success = service_manager_->register_ros_service(service_name, service_type);
  
  if (success) {
    RCLCPP_INFO(get_logger(), "Registered ROS service client: %s [%s]",
                service_name.c_str(), service_type.c_str());
    send_log_to_client(client_fd, "info",
                      "RegisterRosService(" + service_name + ", " + service_type + ") OK");
  } else {
    send_error_to_client(client_fd, "Failed to register ROS service: " + service_name);
  }
}


void MessageRouter::handle_unity_service_command(int client_fd, const std::string& params)
{
  // Parse JSON parameters
  std::unordered_map<std::string, std::string> param_map;
  if (!parse_json_params(params, param_map)) {
    send_error_to_client(client_fd, "Failed to parse Unity service parameters");
    return;
  }
  
  std::string service_name = param_map["topic"];
  std::string service_type = param_map["message_name"];
  
  if (service_name.empty() || service_type.empty()) {
    send_error_to_client(client_fd, "Missing required parameters: topic, message_name");
    return;
  }
  
  // Register Unity service server (ROS can call this service, Unity responds)
  auto request_callback = [this, client_fd](
    uint32_t srv_id,
    const std::string& service_name,
    const std::vector<uint8_t>& request)
  {
    // Send request to Unity client
    if (send_callback_) {
      // Send __request command with srv_id first
      nlohmann::json req_cmd;
      req_cmd["srv_id"] = srv_id;
      std::string json_str = req_cmd.dump();
      std::vector<uint8_t> cmd_data(json_str.begin(), json_str.end());
      send_callback_(client_fd, "__request", cmd_data);
      
      // Then send the actual request data
      send_callback_(client_fd, service_name, request);
    }
  };
  
  bool success = service_manager_->register_unity_service(
    service_name, service_type, request_callback);
  
  if (success) {
    RCLCPP_INFO(get_logger(), "Registered Unity service server: %s [%s]",
                service_name.c_str(), service_type.c_str());
    send_log_to_client(client_fd, "info",
                      "RegisterUnityService(" + service_name + ", " + service_type + ") OK");
  } else {
    send_error_to_client(client_fd, "Failed to register Unity service: " + service_name);
  }
}

void MessageRouter::handle_request_command(int client_fd, const std::string& params)
{
  // Parse srv_id from JSON
  std::unordered_map<std::string, std::string> param_map;
  if (!parse_json_params(params, param_map)) {
    send_error_to_client(client_fd, "Failed to parse request parameters");
    return;
  }
  
  uint32_t srv_id = std::stoul(param_map["srv_id"]);
  
  // Store pending service request ID
  // The next non-system message will be the service request data
  pending_service_request_id_ = srv_id;
  
  // Remember which client should receive the response
  {
    std::lock_guard<std::mutex> lock(service_response_mutex_);
    service_response_client_[srv_id] = client_fd;
  }
  
  RCLCPP_DEBUG(get_logger(), "Request command received [ID: %u]", srv_id);
}

void MessageRouter::handle_response_command(int client_fd, const std::string& params)
{
  // Parse srv_id from JSON
  std::unordered_map<std::string, std::string> param_map;
  if (!parse_json_params(params, param_map)) {
    send_error_to_client(client_fd, "Failed to parse response parameters");
    return;
  }
  
  uint32_t srv_id = std::stoul(param_map["srv_id"]);
  
  // Store pending service response ID
  // The next non-system message will be the service response data
  pending_service_response_id_ = srv_id;
  
  RCLCPP_DEBUG(get_logger(), "Response command received [ID: %u]", srv_id);
}


void MessageRouter::handle_topic_list_command(int client_fd)
{
  // Get all ROS topics
  auto topic_names_and_types = get_topic_names_and_types();
  
  // Build response
  nlohmann::json response;
  response["topics"] = nlohmann::json::array();
  response["types"] = nlohmann::json::array();
  
  for (const auto& [topic, types] : topic_names_and_types) {
    response["topics"].push_back(topic);
    if (!types.empty()) {
      response["types"].push_back(types[0]);
    } else {
      response["types"].push_back("unknown");
    }
  }
  
  std::string json_str = response.dump();
  
  // Send as system command response
  if (send_callback_) {
    std::vector<uint8_t> data(json_str.begin(), json_str.end());
    send_callback_(client_fd, "__topic_list", data);
  }
  
  RCLCPP_DEBUG(get_logger(), "Sent topic list with %zu topics", 
               topic_names_and_types.size());
}

void MessageRouter::handle_handshake_command(int client_fd)
{
  send_handshake(client_fd);
}

void MessageRouter::send_handshake(int client_fd)
{
  if (!send_callback_) {
    return;
  }
  
  nlohmann::json handshake;
  handshake["version"] = "v0.7.0";
  handshake["metadata"]["protocol"] = "ROS2";
  handshake["metadata"]["bridge"] = "HORUS";
  
  std::string json_str = handshake.dump();
  std::vector<uint8_t> data(json_str.begin(), json_str.end());
  send_callback_(client_fd, "__handshake", data);
  RCLCPP_INFO(get_logger(), "Sent handshake to client %d", client_fd);
}

void MessageRouter::send_error_to_client(int client_fd, const std::string& error_message)
{
  send_log_to_client(client_fd, "error", error_message);
}

void MessageRouter::send_log_to_client(int client_fd, const std::string& level,
                                      const std::string& message)
{
  if (!send_callback_) {
    return;
  }
  
  nlohmann::json log_msg;
  log_msg["text"] = message;
  
  std::string json_str = log_msg.dump();
  std::string command = "__" + level;
  
  std::vector<uint8_t> data(json_str.begin(), json_str.end());
  send_callback_(client_fd, command, data);
}

bool MessageRouter::parse_json_params(const std::string& json_str,
                                     std::unordered_map<std::string, std::string>& params)
{
  try {
    // Remove trailing null character if present
    std::string cleaned_json = json_str;
    if (!cleaned_json.empty() && cleaned_json.back() == '\0') {
      cleaned_json.pop_back();
    }
    
    auto json_obj = nlohmann::json::parse(cleaned_json);
    
    for (auto& [key, value] : json_obj.items()) {
      if (value.is_string()) {
        params[key] = value.get<std::string>();
      } else {
        params[key] = value.dump();
      }
    }
    
    return true;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "Failed to parse JSON: %s", e.what());
    return false;
  }
}

} // namespace horus_unity_bridge
