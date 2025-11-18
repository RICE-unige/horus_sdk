// SPDX-FileCopyrightText: 2025 RICE Lab, University of Genoa
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
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
  std::memcpy(rcl_msg.buffer, data.data(), data.size());
  rcl_msg.buffer_length = data.size();
}
}  // namespace detail


/**
 * @brief Manages ROS2 service calls between Unity and ROS
 * 
 * Features:
 * - Typed async ROS service calls (Unity -> ROS)
 * - Unity service servers (ROS -> Unity) with typed support
 * - Request/response ID matching
 * - Timeout handling (10s default)
 * - Support for common service types (AddTwoInts, Trigger, SetBool)
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

private:
  rclcpp::Node* node_;
  
  // Base class for type-erased service client wrappers
  struct RosServiceClientBase {
    virtual ~RosServiceClientBase() = default;
    virtual bool is_ready() const = 0;
    virtual bool call_async(uint32_t srv_id, const std::vector<uint8_t>& request) = 0;
    virtual bool check_response(uint32_t srv_id, std::vector<uint8_t>& response) = 0;
    virtual bool has_timedout(uint32_t srv_id) const = 0;
    virtual void cleanup(uint32_t srv_id) = 0;
  };
  
  // Typed service client wrapper
  template<typename ServiceT>
  struct TypedServiceClient : public RosServiceClientBase {
    typename rclcpp::Client<ServiceT>::SharedPtr client;
    struct PendingCall {
      std::shared_future<typename ServiceT::Response::SharedPtr> future;
      rclcpp::Time start_time;
    };
    std::unordered_map<uint32_t, PendingCall> pending;
    rclcpp::Node* node;
    
    TypedServiceClient(typename rclcpp::Client<ServiceT>::SharedPtr c, rclcpp::Node* n)
      : client(c), node(n) {}
    
    bool is_ready() const override { return client->service_is_ready(); }
    
    bool call_async(uint32_t srv_id, const std::vector<uint8_t>& request) override {
      try {
        // Deserialize request using introspection type support
        auto req = std::make_shared<typename ServiceT::Request>();
        std::vector<uint8_t> normalized_request;
        const std::vector<uint8_t>* payload = &request;
        if (!detail::has_cdr_header(request)) {
          normalized_request = detail::add_cdr_header(request);
          payload = &normalized_request;
        }
        rclcpp::SerializedMessage serialized_msg;
        detail::fill_serialized_message(*payload, serialized_msg);

        // Deserialize using default type support
        rclcpp::Serialization<typename ServiceT::Request> serializer;
        serializer.deserialize_message(&serialized_msg, req.get());
        
        // Call service async
        auto future = client->async_send_request(req);
        
        PendingCall call;
        call.future = future.share();
        call.start_time = node->now();
        pending[srv_id] = call;
        
        return true;
      } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Failed to deserialize service request: %s", e.what());
        return false;
      }
    }
    
    bool check_response(uint32_t srv_id, std::vector<uint8_t>& response) override {
      auto it = pending.find(srv_id);
      if (it == pending.end()) return false;
      
      if (it->second.future.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
        try {
          auto resp = it->second.future.get();
          
          // Serialize response using default type support
          rclcpp::Serialization<typename ServiceT::Response> serializer;
          rclcpp::SerializedMessage serialized_msg;
          serializer.serialize_message(resp.get(), &serialized_msg);
          
          auto& rcl_msg = serialized_msg.get_rcl_serialized_message();
          response.assign(rcl_msg.buffer, rcl_msg.buffer + rcl_msg.buffer_length);
          
          pending.erase(it);
          return true;
        } catch (const std::exception& e) {
          RCLCPP_ERROR(node->get_logger(), "Failed to serialize service response: %s", e.what());
          pending.erase(it);
          return false;
        }
      }
      return false;
    }
    
    bool has_timedout(uint32_t srv_id) const override {
      auto it = pending.find(srv_id);
      if (it == pending.end()) return false;
      auto elapsed = node->now() - it->second.start_time;
      return elapsed.seconds() > 10.0;
    }
    
    void cleanup(uint32_t srv_id) override {
      pending.erase(srv_id);
    }
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
  
  std::unordered_map<std::string, std::shared_ptr<RosServiceClientBase>> ros_service_clients_;
  std::unordered_map<uint32_t, std::string> pending_service_names_;  // srv_id -> service_name
  std::mutex ros_clients_mutex_;
  
  std::unordered_map<std::string, std::shared_ptr<UnityServiceServerBase>> unity_service_servers_;
  std::unordered_map<uint32_t, std::string> pending_unity_service_names_;  // srv_id -> service_name
  std::mutex unity_servers_mutex_;
  
  uint32_t next_service_id_ = 1000;
  
  RosServiceResponseCallback ros_response_callback_;
  
  ServiceStatistics stats_;
  
  // Factory methods for creating typed clients/servers
  std::shared_ptr<RosServiceClientBase> create_service_client(const std::string& service_name,
                                                              const std::string& service_type);
  
  std::shared_ptr<UnityServiceServerBase> create_unity_service_server(
    const std::string& service_name,
    const std::string& service_type,
    UnityServiceRequestCallback callback);
};

} // namespace horus_unity_bridge
