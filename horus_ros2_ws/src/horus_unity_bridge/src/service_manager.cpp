// SPDX-FileCopyrightText: 2025 RICE Lab, University of Genoa
// SPDX-License-Identifier: Apache-2.0

#include "horus_unity_bridge/service_manager.hpp"
#include <rmw/serialized_message.h>
#include <rcutils/allocator.h>

namespace horus_unity_bridge
{

ServiceManager::ServiceManager(rclcpp::Node* node)
  : node_(node)
{
  stats_ = ServiceStatistics{};
}

bool ServiceManager::register_ros_service(const std::string& service_name,
                                         const std::string& service_type)
{
  std::lock_guard<std::mutex> lock(ros_clients_mutex_);
  
  if (ros_service_clients_.find(service_name) != ros_service_clients_.end()) {
    // Already registered, assume same type
    return true;
  }

  std::string normalized_type = normalize_service_type(service_type);
  
  try {
    // Create generic client
    auto client = node_->create_generic_client(service_name, normalized_type, rclcpp::ServicesQoS());
    auto wrapper = std::make_shared<GenericServiceClientWrapper>(client);
    
    ros_service_clients_[service_name] = wrapper;
    
    RCLCPP_INFO(node_->get_logger(), "Registered generic ROS service client: %s [%s]",
                service_name.c_str(), normalized_type.c_str());
    return true;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to create generic client for %s: %s", 
                 service_name.c_str(), e.what());
    stats_.errors++;
    return false;
  }
}

bool ServiceManager::call_ros_service_async(uint32_t srv_id,
                                           const std::string& service_name,
                                           const std::vector<uint8_t>& request)
{
  std::shared_ptr<GenericServiceClientWrapper> wrapper;
  {
    std::lock_guard<std::mutex> lock(ros_clients_mutex_);
    auto it = ros_service_clients_.find(service_name);
    if (it == ros_service_clients_.end()) {
      RCLCPP_ERROR(node_->get_logger(), "Service not registered: %s", service_name.c_str());
      stats_.errors++;
      return false;
    }
    wrapper = it->second;
  }
  
  if (!wrapper->client->service_is_ready()) {
    // Don't log warn every time, just fail
    stats_.errors++;
    return false;
  }
  
  try {
    // Prepare serialized request using low-level C struct with custom deleter
    std::shared_ptr<rcl_serialized_message_t> req_msg(
        new rcl_serialized_message_t(rmw_get_zero_initialized_serialized_message()),
        [](rcl_serialized_message_t* msg) {
            if (rmw_serialized_message_fini(msg) != RMW_RET_OK) {
                // In a deleter we can't easily log to the node without capturing it, 
                // but we should avoid memory leaks.
                // rmw_serialized_message_fini frees the internal buffer.
            }
            delete msg;
        }
    );
    
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    
    std::vector<uint8_t> normalized_request;
    const std::vector<uint8_t>* payload = &request;
    
    // Ensure CDR header if missing
    if (!detail::has_cdr_header(request)) {
      normalized_request = detail::add_cdr_header(request);
      payload = &normalized_request;
    }
    
    if (rmw_serialized_message_init(req_msg.get(), payload->size(), &allocator) != RMW_RET_OK) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to init serialized message");
        return false;
    }
    
    std::memcpy(req_msg->buffer, payload->data(), payload->size());
    req_msg->buffer_length = payload->size();
    
    // Call generic service - pass raw pointer as per Jazzy void* signature
    // We cast to prevent type mismatch, assuming GenericClient takes ownership or copies
    // Note: async_send_request(Request request). If Request is void*, it's likely a type-erased handle.
    // However, Generic Client usually copies the data to DDS.
    auto future = wrapper->client->async_send_request(req_msg.get());
    
    GenericServiceClientWrapper::PendingCall call;
    call.future = future.share();
    call.start_time = node_->now();
    
    wrapper->pending[srv_id] = call;
    
    {
      std::lock_guard<std::mutex> lock(ros_clients_mutex_);
      pending_service_names_[srv_id] = service_name;
    }
    
    stats_.ros_services_called++;
    RCLCPP_DEBUG(node_->get_logger(), "Called generic ROS service: %s [ID: %u]",
                 service_name.c_str(), srv_id);
    
    return true;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to call generic service %s: %s",
                 service_name.c_str(), e.what());
    stats_.errors++;
    return false;
  }
}

void ServiceManager::process_pending_responses()
{
  std::lock_guard<std::mutex> lock(ros_clients_mutex_);
  
  std::vector<uint32_t> to_remove;
  
  // Iterate strictly over pending_service_names to find active calls
  for (auto& [srv_id, service_name] : pending_service_names_) {
    auto it = ros_service_clients_.find(service_name);
    if (it == ros_service_clients_.end()) {
      to_remove.push_back(srv_id);
      continue;
    }
    
    auto& wrapper = it->second;
    auto call_it = wrapper->pending.find(srv_id);
    if (call_it == wrapper->pending.end()) {
      to_remove.push_back(srv_id);
      continue;
    }
    
    auto& call = call_it->second;
    
    // Check if future is ready
    if (call.future.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
      try {
        auto void_resp = call.future.get();
        auto resp_msg = std::static_pointer_cast<rcl_serialized_message_t>(void_resp);
        
        // Convert back to vector
        std::vector<uint8_t> response(resp_msg->buffer, resp_msg->buffer + resp_msg->buffer_length);
        
        if (ros_response_callback_) {
          ros_response_callback_(srv_id, response);
        }
        stats_.responses_sent++;
      } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Generic service call failed: %s", e.what());
        stats_.errors++;
      }
      wrapper->pending.erase(call_it);
      to_remove.push_back(srv_id);
    } 
    else {
      // Check timeout
      auto elapsed = node_->now() - call.start_time;
      if (elapsed.seconds() > 10.0) {
        RCLCPP_WARN(node_->get_logger(), "Service call timeout [ID: %u]", srv_id);
        wrapper->pending.erase(call_it);
        stats_.timeouts++;
        to_remove.push_back(srv_id);
      }
    }
  }
  
  for (auto srv_id : to_remove) {
    pending_service_names_.erase(srv_id);
  }
}

bool ServiceManager::register_unity_service(const std::string& service_name,
                                           const std::string& service_type,
                                           UnityServiceRequestCallback request_callback)
{
  std::lock_guard<std::mutex> lock(unity_servers_mutex_);
  
  if (unity_service_servers_.find(service_name) != unity_service_servers_.end()) {
    return true; 
  }
  
  auto server = create_unity_service_server(service_name, service_type, request_callback);
  
  if (!server) {
    stats_.errors++;
    return false;
  }
  
  unity_service_servers_[service_name] = server;
    
  RCLCPP_INFO(node_->get_logger(), "Registered Typed Unity service server: %s [%s]",
              service_name.c_str(), service_type.c_str());
  return true;
}

void ServiceManager::handle_unity_service_response(uint32_t srv_id,
                                                  const std::vector<uint8_t>& response)
{
  std::lock_guard<std::mutex> lock(unity_servers_mutex_);
  
  auto name_it = pending_unity_service_names_.find(srv_id);
  if (name_it == pending_unity_service_names_.end()) {
    RCLCPP_WARN(node_->get_logger(), "Received response for unknown/expired Unity service ID: %u", srv_id);
    return;
  }
  
  std::string service_name = name_it->second;
  auto server_it = unity_service_servers_.find(service_name);
  if (server_it == unity_service_servers_.end()) {
    RCLCPP_ERROR(node_->get_logger(), "Unity service server not found: %s", service_name.c_str());
    pending_unity_service_names_.erase(name_it);
    return;
  }
  
  // Try to handle as AddTwoInts
  auto add_two_ints_server = std::dynamic_pointer_cast<TypedUnityServiceServer<example_interfaces::srv::AddTwoInts>>(
    server_it->second);
  if (add_two_ints_server) {
    std::lock_guard<std::mutex> srv_lock(add_two_ints_server->mutex);
    auto it = add_two_ints_server->pending.find(srv_id);
    if (it != add_two_ints_server->pending.end()) {
      // Deserialize response
      auto resp = std::make_shared<example_interfaces::srv::AddTwoInts::Response>();
      std::vector<uint8_t> normalized;
      const std::vector<uint8_t>* payload = &response;
      if (!detail::has_cdr_header(response)) {
        normalized = detail::add_cdr_header(response);
        payload = &normalized;
      }
      rclcpp::SerializedMessage serialized_msg;
      detail::fill_serialized_message(*payload, serialized_msg);
      
      rclcpp::Serialization<example_interfaces::srv::AddTwoInts::Response> serializer;
      serializer.deserialize_message(&serialized_msg, resp.get());
      
      it->second.set_value(resp);
      add_two_ints_server->pending.erase(it);
      stats_.responses_sent++;
      pending_unity_service_names_.erase(name_it);
      return;
    }
  }
  
  // Try Trigger
  auto trigger_server = std::dynamic_pointer_cast<TypedUnityServiceServer<std_srvs::srv::Trigger>>(
    server_it->second);
  if (trigger_server) {
    std::lock_guard<std::mutex> srv_lock(trigger_server->mutex);
    auto it = trigger_server->pending.find(srv_id);
    if (it != trigger_server->pending.end()) {
      auto resp = std::make_shared<std_srvs::srv::Trigger::Response>();
      std::vector<uint8_t> normalized;
      const std::vector<uint8_t>* payload = &response;
      if (!detail::has_cdr_header(response)) {
        normalized = detail::add_cdr_header(response);
        payload = &normalized;
      }
      rclcpp::SerializedMessage serialized_msg;
      detail::fill_serialized_message(*payload, serialized_msg);
      
      rclcpp::Serialization<std_srvs::srv::Trigger::Response> serializer;
      serializer.deserialize_message(&serialized_msg, resp.get());
      
      it->second.set_value(resp);
      trigger_server->pending.erase(it);
      stats_.responses_sent++;
      pending_unity_service_names_.erase(name_it);
      return;
    }
  }

  // Try SetBool
  auto set_bool_server = std::dynamic_pointer_cast<TypedUnityServiceServer<std_srvs::srv::SetBool>>(
    server_it->second);
  if (set_bool_server) {
    std::lock_guard<std::mutex> srv_lock(set_bool_server->mutex);
    auto it = set_bool_server->pending.find(srv_id);
    if (it != set_bool_server->pending.end()) {
      auto resp = std::make_shared<std_srvs::srv::SetBool::Response>();
      std::vector<uint8_t> normalized;
      const std::vector<uint8_t>* payload = &response;
      if (!detail::has_cdr_header(response)) {
        normalized = detail::add_cdr_header(response);
        payload = &normalized;
      }
      rclcpp::SerializedMessage serialized_msg;
      detail::fill_serialized_message(*payload, serialized_msg);
      
      rclcpp::Serialization<std_srvs::srv::SetBool::Response> serializer;
      serializer.deserialize_message(&serialized_msg, resp.get());
      
      it->second.set_value(resp);
      set_bool_server->pending.erase(it);
      stats_.responses_sent++;
      pending_unity_service_names_.erase(name_it);
      return;
    }
  }
  
  RCLCPP_ERROR(node_->get_logger(), "Failed to handle Unity service response [ID: %u]", srv_id);
}

std::shared_ptr<ServiceManager::UnityServiceServerBase> ServiceManager::create_unity_service_server(
  const std::string& service_name,
  const std::string& service_type,
  UnityServiceRequestCallback callback)
{
  // Factory for creating typed Unity service servers
  if (service_type.find("AddTwoInts") != std::string::npos) {
    auto service_callback = [this, callback, service_name](
      const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
      std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response)
    {
      // Generate unique service ID
      uint32_t srv_id;
      {
        std::lock_guard<std::mutex> lock(unity_servers_mutex_);
        srv_id = next_service_id_++;
        pending_unity_service_names_[srv_id] = service_name;
      }
      
      // Serialize request
      rclcpp::Serialization<example_interfaces::srv::AddTwoInts::Request> serializer;
      rclcpp::SerializedMessage serialized_msg;
      serializer.serialize_message(request.get(), &serialized_msg);
      
      auto& rcl_msg = serialized_msg.get_rcl_serialized_message();
      std::vector<uint8_t> request_data(rcl_msg.buffer, rcl_msg.buffer + rcl_msg.buffer_length);
      
      // Create promise for response
      std::promise<example_interfaces::srv::AddTwoInts::Response::SharedPtr> response_promise;
      auto response_future = response_promise.get_future();
      
      // Store pending call
      auto server_ptr = std::static_pointer_cast<TypedUnityServiceServer<example_interfaces::srv::AddTwoInts>>(
        unity_service_servers_[service_name]);
      {
        std::lock_guard<std::mutex> lock(server_ptr->mutex);
        server_ptr->pending[srv_id] = std::move(response_promise);
      }
      
      stats_.unity_services_called++;
      
      // Send request to Unity
      if (callback) {
        callback(srv_id, service_name, request_data);
      }
      
      // Wait for Unity response (with timeout)
      auto status = response_future.wait_for(std::chrono::seconds(10));
      
      if (status == std::future_status::ready) {
        auto resp = response_future.get();
        *response = *resp;
      } else {
        RCLCPP_ERROR(node_->get_logger(), "Unity service timeout: %s [ID: %u]",
                     service_name.c_str(), srv_id);
        stats_.timeouts++;
        
        // Clean up
        std::lock_guard<std::mutex> lock(unity_servers_mutex_);
        pending_unity_service_names_.erase(srv_id);
      }
    };
    
    auto server = node_->create_service<example_interfaces::srv::AddTwoInts>(
      service_name, service_callback);
    
    auto unity_server = std::make_shared<TypedUnityServiceServer<example_interfaces::srv::AddTwoInts>>(
      server, callback);
    
    return unity_server;
  } else if (service_type.find("Trigger") != std::string::npos) {
    auto service_callback = [this, callback, service_name](
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
      uint32_t srv_id;
      {
        std::lock_guard<std::mutex> lock(unity_servers_mutex_);
        srv_id = next_service_id_++;
        pending_unity_service_names_[srv_id] = service_name;
      }
      
      rclcpp::Serialization<std_srvs::srv::Trigger::Request> serializer;
      rclcpp::SerializedMessage serialized_msg;
      serializer.serialize_message(request.get(), &serialized_msg);
      
      auto& rcl_msg = serialized_msg.get_rcl_serialized_message();
      std::vector<uint8_t> request_data(rcl_msg.buffer, rcl_msg.buffer + rcl_msg.buffer_length);
      
      std::promise<std_srvs::srv::Trigger::Response::SharedPtr> response_promise;
      auto response_future = response_promise.get_future();
      
      auto server_ptr = std::static_pointer_cast<TypedUnityServiceServer<std_srvs::srv::Trigger>>(
        unity_service_servers_[service_name]);
      {
        std::lock_guard<std::mutex> lock(server_ptr->mutex);
        server_ptr->pending[srv_id] = std::move(response_promise);
      }
      
      stats_.unity_services_called++;
      
      if (callback) {
        callback(srv_id, service_name, request_data);
      }
      
      auto status = response_future.wait_for(std::chrono::seconds(10));
      
      if (status == std::future_status::ready) {
        auto resp = response_future.get();
        *response = *resp;
      } else {
        RCLCPP_ERROR(node_->get_logger(), "Unity service timeout: %s [ID: %u]",
                     service_name.c_str(), srv_id);
        stats_.timeouts++;
        std::lock_guard<std::mutex> lock(unity_servers_mutex_);
        pending_unity_service_names_.erase(srv_id);
      }
    };
    
    auto server = node_->create_service<std_srvs::srv::Trigger>(service_name, service_callback);
    return std::make_shared<TypedUnityServiceServer<std_srvs::srv::Trigger>>(server, callback);
  } else if (service_type.find("SetBool") != std::string::npos) {
    auto service_callback = [this, callback, service_name](
      const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
      std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
      uint32_t srv_id;
      {
        std::lock_guard<std::mutex> lock(unity_servers_mutex_);
        srv_id = next_service_id_++;
        pending_unity_service_names_[srv_id] = service_name;
      }
      
      rclcpp::Serialization<std_srvs::srv::SetBool::Request> serializer;
      rclcpp::SerializedMessage serialized_msg;
      serializer.serialize_message(request.get(), &serialized_msg);
      
      auto& rcl_msg = serialized_msg.get_rcl_serialized_message();
      std::vector<uint8_t> request_data(rcl_msg.buffer, rcl_msg.buffer + rcl_msg.buffer_length);
      
      std::promise<std_srvs::srv::SetBool::Response::SharedPtr> response_promise;
      auto response_future = response_promise.get_future();
      
      auto server_ptr = std::static_pointer_cast<TypedUnityServiceServer<std_srvs::srv::SetBool>>(
        unity_service_servers_[service_name]);
      {
        std::lock_guard<std::mutex> lock(server_ptr->mutex);
        server_ptr->pending[srv_id] = std::move(response_promise);
      }
      
      stats_.unity_services_called++;
      
      if (callback) {
        callback(srv_id, service_name, request_data);
      }
      
      auto status = response_future.wait_for(std::chrono::seconds(10));
      
      if (status == std::future_status::ready) {
        auto resp = response_future.get();
        *response = *resp;
      } else {
        RCLCPP_ERROR(node_->get_logger(), "Unity service timeout: %s [ID: %u]",
                     service_name.c_str(), srv_id);
        stats_.timeouts++;
        std::lock_guard<std::mutex> lock(unity_servers_mutex_);
        pending_unity_service_names_.erase(srv_id);
      }
    };
    
    auto server = node_->create_service<std_srvs::srv::SetBool>(service_name, service_callback);
    return std::make_shared<TypedUnityServiceServer<std_srvs::srv::SetBool>>(server, callback);
  }
  
  RCLCPP_ERROR(node_->get_logger(), "Unsupported Unity service type: %s", service_type.c_str());
  return nullptr;
}

std::string ServiceManager::normalize_service_type(const std::string& type)
{
  // Same logic as TopicManager but ensuring /srv/
  std::string normalized = type;
  if (normalized.find("/srv/") != std::string::npos) {
    return normalized;
  }
  size_t slash_pos = normalized.find('/');
  if (slash_pos != std::string::npos) {
     return normalized.substr(0, slash_pos) + "/srv/" + normalized.substr(slash_pos + 1);
  }
  return normalized;
}

} // namespace horus_unity_bridge
