// SPDX-FileCopyrightText: 2025 RICE Lab, University of Genoa
// SPDX-License-Identifier: Apache-2.0

#include "horus_unity_bridge/message_router.hpp"
#include <sstream>
#include <nlohmann/json.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <rclcpp/qos.hpp>
#include <algorithm>
#include <cctype>
#include <cstring>

#ifdef ENABLE_WEBRTC
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#endif

namespace horus_unity_bridge
{

MessageRouter::MessageRouter()
  : Node("horus_unity_bridge")
{
  auto webrtc_enabled_param = this->declare_parameter<bool>("webrtc.enabled", false);
  auto webrtc_camera_topic_param = this->declare_parameter<std::string>("webrtc.camera_topic", "");
  auto webrtc_bitrate_param = this->declare_parameter<int>("webrtc.bitrate_kbps", 2000);
  auto webrtc_framerate_param = this->declare_parameter<int>("webrtc.framerate", 30);
  auto webrtc_encoder_param = this->declare_parameter<std::string>("webrtc.encoder", "x264enc");
  auto webrtc_pipeline_param = this->declare_parameter<std::string>("webrtc.pipeline", "");
  auto webrtc_stun_param = this->declare_parameter<std::vector<std::string>>(
    "webrtc.stun_servers", std::vector<std::string>{"stun:stun.l.google.com:19302"});
  
  topic_manager_ = std::make_unique<TopicManager>(this);
  service_manager_ = std::make_unique<ServiceManager>(this);

#ifdef ENABLE_WEBRTC
  webrtc_enabled_param_ = webrtc_enabled_param;
  webrtc_camera_topic_ = webrtc_camera_topic_param;
  webrtc_bitrate_kbps_ = webrtc_bitrate_param;
  webrtc_framerate_ = webrtc_framerate_param;
  webrtc_encoder_ = webrtc_encoder_param;
  webrtc_pipeline_ = webrtc_pipeline_param;
  webrtc_stun_servers_ = webrtc_stun_param;
  
  if (webrtc_enabled_param_) {
    webrtc_manager_ = std::make_unique<WebRTCManager>();
    WebRTCManager::Settings settings;
    settings.stun_servers = webrtc_stun_servers_;
    settings.encoder = webrtc_encoder_;
    settings.bitrate_kbps = webrtc_bitrate_kbps_;
    settings.framerate = webrtc_framerate_;
    settings.pipeline = webrtc_pipeline_;
    
    if (!webrtc_manager_->initialize(settings)) {
      RCLCPP_WARN(get_logger(), "Failed to initialize WebRTC manager, disabling WebRTC features.");
      webrtc_manager_.reset();
    } else {
      webrtc_manager_->set_signaling_callback(
        [this](const std::string& type, const nlohmann::json& data) {
          if (!broadcast_callback_) {
            RCLCPP_WARN(get_logger(), "WebRTC signaling callback received but no broadcast callback is set.");
            return;
          }
          std::string command;
          if (type == "offer") command = "__webrtc_offer";
          else if (type == "answer") command = "__webrtc_answer";
          else if (type == "candidate") command = "__webrtc_candidate";
          
          if (!command.empty()) {
            std::string json_str = data.dump();
            std::vector<uint8_t> payload(json_str.begin(), json_str.end());
            broadcast_callback_(command, payload);
          }
        }
      );
      
      if (!webrtc_camera_topic_.empty()) {
        setup_camera_subscription(webrtc_camera_topic_);
      }
    }
  } else if (!webrtc_camera_topic_.empty()) {
    RCLCPP_WARN(get_logger(), "Parameter 'webrtc.camera_topic' is set but WebRTC streaming is disabled.");
  }
#else
  if (webrtc_enabled_param) {
    RCLCPP_WARN(get_logger(), "webrtc.enabled parameter is true but this build was compiled without WebRTC support.");
  }
  (void)webrtc_camera_topic_param;
  (void)webrtc_bitrate_param;
  (void)webrtc_framerate_param;
  (void)webrtc_encoder_param;
  (void)webrtc_pipeline_param;
  (void)webrtc_stun_param;
#endif
  
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
  
#ifdef ENABLE_WEBRTC
  if (webrtc_enabled_param_) {
    processing_running_ = true;
    worker_thread_ = std::thread(&MessageRouter::process_frames, this);
  }
#endif

  RCLCPP_INFO(get_logger(), "Message router initialized");
}

MessageRouter::~MessageRouter()
{
#ifdef ENABLE_WEBRTC
  processing_running_ = false;
  frame_queue_cv_.notify_all();
  if (worker_thread_.joinable()) {
    worker_thread_.join();
  }
#endif
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
#ifdef ENABLE_WEBRTC
    } else if (command == "__webrtc_offer") {
      handle_webrtc_offer(client_fd, params_json);
    } else if (command == "__webrtc_answer") {
      handle_webrtc_answer(client_fd, params_json);
    } else if (command == "__webrtc_candidate") {
      handle_webrtc_candidate(client_fd, params_json);
#endif
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

#ifdef ENABLE_WEBRTC
void MessageRouter::handle_webrtc_offer(int client_fd, const std::string& params)
{
  if (!webrtc_manager_) {
    send_error_to_client(client_fd, "WebRTC support disabled on server");
    return;
  }
  std::unordered_map<std::string, std::string> param_map;
  if (!parse_json_params(params, param_map)) {
    return;
  }
  
  auto it = param_map.find("sdp");
  if (it != param_map.end()) {
    webrtc_manager_->handle_offer(it->second);
  }
}

void MessageRouter::handle_webrtc_answer(int client_fd, const std::string& params)
{
  if (!webrtc_manager_) {
    send_error_to_client(client_fd, "WebRTC support disabled on server");
    return;
  }
  std::unordered_map<std::string, std::string> param_map;
  if (!parse_json_params(params, param_map)) {
    return;
  }
  
  auto it = param_map.find("sdp");
  if (it != param_map.end()) {
    webrtc_manager_->handle_answer(it->second);
  }
}

void MessageRouter::handle_webrtc_candidate(int client_fd, const std::string& params)
{
  if (!webrtc_manager_) {
    send_error_to_client(client_fd, "WebRTC support disabled on server");
    return;
  }
  std::unordered_map<std::string, std::string> param_map;
  if (!parse_json_params(params, param_map)) {
    return;
  }
  
  if (!param_map.count("candidate") || !param_map.count("sdpMid")) {
    send_error_to_client(client_fd, "Missing required ICE candidate fields");
    return;
  }
  
  int sdp_mline_index = 0;
  if (param_map.count("sdpMLineIndex")) {
    try {
      sdp_mline_index = std::stoi(param_map["sdpMLineIndex"]);
    } catch (const std::exception& e) {
      RCLCPP_WARN(get_logger(), "Invalid sdpMLineIndex value, using 0: %s", e.what());
    }
  }
  
  webrtc_manager_->handle_candidate(param_map["candidate"], param_map["sdpMid"], sdp_mline_index);
}

void MessageRouter::setup_camera_subscription(const std::string& topic)
{
  if (!webrtc_manager_) {
    return;
  }
  
  auto qos = rclcpp::SensorDataQoS();
  
  // Try to detect if topic is compressed by checking topic name or trying both
  // First, attempt compressed subscription (common pattern: /camera/image/compressed)
  if (topic.find("/compressed") != std::string::npos || topic.find("_compressed") != std::string::npos) {
    compressed_camera_sub_ = create_subscription<sensor_msgs::msg::CompressedImage>(
      topic, qos,
      [this](const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
        handle_compressed_camera_frame(msg);
      });
    RCLCPP_INFO(get_logger(), "WebRTC streaming subscribed to compressed image topic: %s", topic.c_str());
  } else {
    // Subscribe to raw image topic
    camera_sub_ = create_subscription<sensor_msgs::msg::Image>(
      topic, qos,
      [this](const sensor_msgs::msg::Image::SharedPtr msg) {
        handle_camera_frame(msg);
      });
    RCLCPP_INFO(get_logger(), "WebRTC streaming subscribed to raw image topic: %s", topic.c_str());
  }
}

void MessageRouter::handle_camera_frame(const sensor_msgs::msg::Image::SharedPtr& msg)
{
  if (!webrtc_manager_) {
    return;
  }
  
  {
    std::lock_guard<std::mutex> lock(frame_queue_mutex_);
    if (frame_queue_.size() >= max_queue_size_) {
      // Drop oldest frame to keep latency low
      frame_queue_.pop_front();
    }
    frame_queue_.push_back(msg);
  }
  frame_queue_cv_.notify_one();
}

void MessageRouter::handle_compressed_camera_frame(const sensor_msgs::msg::CompressedImage::SharedPtr& msg)
{
  if (!webrtc_manager_) {
    return;
  }
  
  {
    std::lock_guard<std::mutex> lock(frame_queue_mutex_);
    if (frame_queue_.size() >= max_queue_size_) {
      // Drop oldest frame
      frame_queue_.pop_front();
    }
    frame_queue_.push_back(msg);
  }
  frame_queue_cv_.notify_one();
}

void MessageRouter::process_frames()
{
  while (processing_running_) {
    FrameVariant frame_variant;
    {
      std::unique_lock<std::mutex> lock(frame_queue_mutex_);
      frame_queue_cv_.wait(lock, [this] {
        return !frame_queue_.empty() || !processing_running_;
      });
      
      if (!processing_running_) break;
      
      frame_variant = frame_queue_.front();
      frame_queue_.pop_front();
    }
    
    // Process frame outside the lock
    std::vector<uint8_t> buffer;
    if (std::holds_alternative<sensor_msgs::msg::Image::SharedPtr>(frame_variant)) {
      auto msg = std::get<sensor_msgs::msg::Image::SharedPtr>(frame_variant);
      if (convert_to_rgb(*msg, buffer)) {
        webrtc_manager_->push_frame(buffer, msg->width, msg->height, "RGB");
      }
    } else {
      auto msg = std::get<sensor_msgs::msg::CompressedImage::SharedPtr>(frame_variant);
      int width = 0, height = 0;
      if (decompress_to_rgb(*msg, buffer, width, height)) {
        webrtc_manager_->push_frame(buffer, width, height, "RGB");
      }
    }
  }
}

bool MessageRouter::convert_to_rgb(const sensor_msgs::msg::Image& msg, std::vector<uint8_t>& output)
{
  const auto width = static_cast<size_t>(msg.width);
  const auto height = static_cast<size_t>(msg.height);
  const auto row_stride = static_cast<size_t>(msg.step);
  if (width == 0 || height == 0 || row_stride == 0) {
    return false;
  }
  
  auto equals_ignore_case = [](const std::string& lhs, const std::string& rhs) {
    if (lhs.size() != rhs.size()) {
      return false;
    }
    for (size_t i = 0; i < lhs.size(); ++i) {
      if (std::tolower(static_cast<unsigned char>(lhs[i])) !=
          std::tolower(static_cast<unsigned char>(rhs[i]))) {
        return false;
      }
    }
    return true;
  };
  
  if (equals_ignore_case(msg.encoding, sensor_msgs::image_encodings::RGB8)) {
    output.resize(width * height * 3);
    for (size_t y = 0; y < height; ++y) {
      const uint8_t* row = msg.data.data() + y * row_stride;
      std::memcpy(output.data() + y * width * 3, row, width * 3);
    }
    return true;
  }
  
  if (equals_ignore_case(msg.encoding, sensor_msgs::image_encodings::BGR8)) {
    output.resize(width * height * 3);
    for (size_t y = 0; y < height; ++y) {
      const uint8_t* row = msg.data.data() + y * row_stride;
      for (size_t x = 0; x < width; ++x) {
        const uint8_t* pixel = row + x * 3;
        size_t dst_index = (y * width + x) * 3;
        output[dst_index] = pixel[2];
        output[dst_index + 1] = pixel[1];
        output[dst_index + 2] = pixel[0];
      }
    }
    return true;
  }
  
  if (equals_ignore_case(msg.encoding, sensor_msgs::image_encodings::MONO8)) {
    output.resize(width * height * 3);
    for (size_t y = 0; y < height; ++y) {
      const uint8_t* row = msg.data.data() + y * row_stride;
      for (size_t x = 0; x < width; ++x) {
        uint8_t value = row[x];
        size_t dst_index = (y * width + x) * 3;
        output[dst_index] = value;
        output[dst_index + 1] = value;
        output[dst_index + 2] = value;
      }
    }
    return true;
  }
  
  if (!warned_unsupported_encoding_) {
    RCLCPP_WARN(get_logger(), "Unsupported camera encoding '%s' for WebRTC streaming", msg.encoding.c_str());
    warned_unsupported_encoding_ = true;
  }
  return false;
}

bool MessageRouter::decompress_to_rgb(const sensor_msgs::msg::CompressedImage& msg, 
                                      std::vector<uint8_t>& output, int& width, int& height)
{
  try {
    cv::Mat buffer_mat(1, static_cast<int>(msg.data.size()), CV_8UC1,
                       const_cast<uint8_t*>(msg.data.data()));
    
    cv::Mat decoded = cv::imdecode(buffer_mat, cv::IMREAD_COLOR);
    
    if (decoded.empty()) {
      if (!warned_unsupported_encoding_) {
        RCLCPP_WARN(get_logger(), "Failed to decompress image with format '%s'", msg.format.c_str());
        warned_unsupported_encoding_ = true;
      }
      return false;
    }
    
    width = decoded.cols;
    height = decoded.rows;
    
    // OpenCV decodes to BGR, convert to RGB
    cv::Mat rgb;
    cv::cvtColor(decoded, rgb, cv::COLOR_BGR2RGB);
    
    // Copy to output buffer
    output.resize(width * height * 3);
    std::memcpy(output.data(), rgb.data, output.size());
    
    return true;
  } catch (const cv::Exception& e) {
    if (!warned_unsupported_encoding_) {
      RCLCPP_ERROR(get_logger(), "OpenCV exception during decompression: %s", e.what());
      warned_unsupported_encoding_ = true;
    }
    return false;
  }
}
#endif

} // namespace horus_unity_bridge
