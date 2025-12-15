// SPDX-FileCopyrightText: 2025 RICE Lab, University of Genoa
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/generic_client.hpp>
// #include <rclcpp/generic_service.hpp> // Not available in Humble
#include <example_interfaces/srv/add_two_ints.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <memory>
#include <string>
#include <vector>
#include <unordered_map>
#include <functional>
#include <mutex>
#include <future>
#include <chrono>
#include <cstring>

namespace horus_unity_bridge
{

namespace detail
{
inline bool has_cdr_header(const std::vector<uint8_t>& data)
{
  if (data.size() < 4) {
    return false;
  }
  if (data[0] != 0x00 || data[2] != 0x00 || data[3] != 0x00) {
    return false;
  }
  return data[1] == 0x00 || data[1] == 0x01;
}

inline std::vector<uint8_t> add_cdr_header(const std::vector<uint8_t>& data)
{
  std::vector<uint8_t> result;
  result.reserve(data.size() + 4);
  result.push_back(0x00);
  result.push_back(0x01);  // little-endian
  result.push_back(0x00);
  result.push_back(0x00);
  result.insert(result.end(), data.begin(), data.end());
  return result;
}

inline void fill_serialized_message(const std::vector<uint8_t>& data,
                                    rclcpp::SerializedMessage& serialized_msg)
{
  serialized_msg.reserve(data.size());
  auto& rcl_msg = serialized_msg.get_rcl_serialized_message();
  // Ensure buffer is large enough
  if (rcl_msg.buffer_capacity < data.size()) {
    rcl_msg.allocator.deallocate(rcl_msg.buffer, rcl_msg.allocator.state);
    rcl_msg.buffer = static_cast<uint8_t*>(rcl_msg.allocator.allocate(data.size(), rcl_msg.allocator.state));
    rcl_msg.buffer_capacity = data.size();
  }
  std::memcpy(rcl_msg.buffer, data.data(), data.size());
  rcl_msg.buffer_length = data.size();
}
}  // namespace detail


/**
 * @brief Manages ROS2 service calls between Unity and ROS using Generic Types
 * 
 * Features:
 * - Generic async ROS service calls (Unity -> ROS) (supports ANY service type)
 * - Generic Unity service servers (ROS -> Unity) (supports ANY service type)
 * - Request/response ID matching
 * - Timeout handling (10s default)
 */
class ServiceManager
{
public:
  // Callback to send requests to Unity for Unity-implemented services
  using UnityServiceRequestCallback = std::function<void(uint32_t srv_id,
                                                         const std::string& service_name,
                                                         const std::vector<uint8_t>& request)>;
  
  // Callback to send ROS service responses back to Unity
  using RosServiceResponseCallback = std::function<void(uint32_t srv_id,
                                                        const std::vector<uint8_t>& response)>;
  
  explicit ServiceManager(rclcpp::Node* node);
  ~ServiceManager() = default;
  
  /**
   * @brief Register a ROS service that Unity can call
   * @param service_name Service name (e.g., "/add_two_ints")
   * @param service_type Service type (e.g., "example_interfaces/srv/AddTwoInts")
   * @return True if registered successfully
   */
  bool register_ros_service(const std::string& service_name,
                           const std::string& service_type);
  
  /**
   * @brief Call a ROS service from Unity (async)
   * @param srv_id Service request ID from Unity
   * @param service_name Service name
   * @param request Serialized service request
   * @return True if call was initiated
   */
  bool call_ros_service_async(uint32_t srv_id,
                             const std::string& service_name,
                             const std::vector<uint8_t>& request);
  
  /**
   * @brief Register a Unity service that ROS can call
   * @param service_name Service name
   * @param service_type Service type
   * @param request_callback Callback to forward request to Unity
   * @return True if registered successfully
   */
  bool register_unity_service(const std::string& service_name,
                             const std::string& service_type,
                             UnityServiceRequestCallback request_callback);
  
  /**
   * @brief Handle service response from Unity
   * @param srv_id Service request ID
   * @param response Serialized service response
   */
  void handle_unity_service_response(uint32_t srv_id,
                                     const std::vector<uint8_t>& response);
  
  /**
   * @brief Set callback for ROS service responses (to send to Unity)
   */
  void set_response_callback(RosServiceResponseCallback callback) {
    ros_response_callback_ = callback;
  }
  
  /**
   * @brief Process pending service responses (called periodically)
   */
  void process_pending_responses();
  
  /**
   * @brief Get service statistics
   */
  struct ServiceStatistics {
    uint64_t ros_services_called = 0;
    uint64_t unity_services_called = 0;
    uint64_t responses_sent = 0;
    uint64_t timeouts = 0;
    uint64_t errors = 0;
  };
  
  const ServiceStatistics& get_statistics() const { return stats_; }
  
  // Clean up all pending service calls (useful on client disconnect if we tracked it)
  // For now, timeouts handle this.

private:
  rclcpp::Node* node_;
  
  // Wrapper for Generic Client
  struct GenericServiceClientWrapper {
    std::shared_ptr<rclcpp::GenericClient> client;
    struct PendingCall {
      std::shared_future<std::shared_ptr<void>> future; 
      rclcpp::Time start_time;
    };
    std::unordered_map<uint32_t, PendingCall> pending;
    
    GenericServiceClientWrapper(std::shared_ptr<rclcpp::GenericClient> c) : client(c) {}
  };
  
  // Base class for Unity service servers
  struct UnityServiceServerBase {
    virtual ~UnityServiceServerBase() = default;
  };
  
  // Typed Unity service server
  template<typename ServiceT>
  struct TypedUnityServiceServer : public UnityServiceServerBase {
    typename rclcpp::Service<ServiceT>::SharedPtr server;
    UnityServiceRequestCallback request_callback;
    std::unordered_map<uint32_t, std::promise<typename ServiceT::Response::SharedPtr>> pending;
    std::mutex mutex;
    
    TypedUnityServiceServer(typename rclcpp::Service<ServiceT>::SharedPtr s,
                           UnityServiceRequestCallback cb)
      : server(s), request_callback(cb) {}
  };
  
  std::unordered_map<std::string, std::shared_ptr<GenericServiceClientWrapper>> ros_service_clients_;
  std::unordered_map<uint32_t, std::string> pending_service_names_;  // srv_id -> service_name
  std::mutex ros_clients_mutex_;
  
  std::unordered_map<std::string, std::shared_ptr<UnityServiceServerBase>> unity_service_servers_;
  std::unordered_map<uint32_t, std::string> pending_unity_service_names_;  // srv_id -> service_name
  std::mutex unity_servers_mutex_;
  
  uint32_t next_service_id_ = 1000;
  
  RosServiceResponseCallback ros_response_callback_;
  
  ServiceStatistics stats_;
  
  // Helper to normalize service types
  std::string normalize_service_type(const std::string& type);
  
  // Factory for Unity services (Typed fallback)
  std::shared_ptr<UnityServiceServerBase> create_unity_service_server(
    const std::string& service_name,
    const std::string& service_type,
    UnityServiceRequestCallback callback);
};

} // namespace horus_unity_bridge
