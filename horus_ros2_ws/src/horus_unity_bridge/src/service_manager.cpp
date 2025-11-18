// SPDX-FileCopyrightText: 2025 RICE Lab, University of Genoa
// SPDX-License-Identifier: Apache-2.0

#include "horus_unity_bridge/service_manager.hpp"

namespace horus_unity_bridge
{

ServiceManager::ServiceManager(rclcpp::Node* node)
  : node_(node)
{
  stats_ = ServiceStatistics{};
}

std::shared_ptr<ServiceManager::RosServiceClientBase> ServiceManager::create_service_client(
  const std::string& service_name,
  const std::string& service_type)
{
  // Factory for creating typed service clients based on service type
  if (service_type == "example_interfaces/srv/AddTwoInts") {
    auto client = node_->create_client<example_interfaces::srv::AddTwoInts>(service_name);
    return std::make_shared<TypedServiceClient<example_interfaces::srv::AddTwoInts>>(client, node_);
  } else if (service_type == "std_srvs/srv/Trigger") {
    auto client = node_->create_client<std_srvs::srv::Trigger>(service_name);
    return std::make_shared<TypedServiceClient<std_srvs::srv::Trigger>>(client, node_);
  } else if (service_type == "std_srvs/srv/SetBool") {
    auto client = node_->create_client<std_srvs::srv::SetBool>(service_name);
    return std::make_shared<TypedServiceClient<std_srvs::srv::SetBool>>(client, node_);
  }
  
  RCLCPP_ERROR(node_->get_logger(), "Unsupported service type: %s", service_type.c_str());
  return nullptr;
}

std::shared_ptr<ServiceManager::UnityServiceServerBase> ServiceManager::create_unity_service_server(
  const std::string& service_name,
  const std::string& service_type,
  UnityServiceRequestCallback callback)
{
  // Factory for creating typed Unity service servers
  if (service_type == "example_interfaces/srv/AddTwoInts") {
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
  } else if (service_type == "std_srvs/srv/Trigger") {
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
  }
  
  RCLCPP_ERROR(node_->get_logger(), "Unsupported Unity service type: %s", service_type.c_str());
  return nullptr;
}

bool ServiceManager::register_ros_service(const std::string& service_name,
                                         const std::string& service_type)
{
  std::lock_guard<std::mutex> lock(ros_clients_mutex_);
  
  if (ros_service_clients_.find(service_name) != ros_service_clients_.end()) {
    RCLCPP_WARN(node_->get_logger(), "ROS service already registered: %s", service_name.c_str());
    return true;
  }
  
  auto client = create_service_client(service_name, service_type);
  if (!client) {
    stats_.errors++;
    return false;
  }
  
  ros_service_clients_[service_name] = client;
  
  RCLCPP_INFO(node_->get_logger(), "Registered ROS service client: %s [%s]",
              service_name.c_str(), service_type.c_str());
  
  return true;
}

bool ServiceManager::call_ros_service_async(uint32_t srv_id,
                                           const std::string& service_name,
                                           const std::vector<uint8_t>& request)
{
  std::shared_ptr<RosServiceClientBase> client;
  {
    std::lock_guard<std::mutex> lock(ros_clients_mutex_);
    auto it = ros_service_clients_.find(service_name);
    if (it == ros_service_clients_.end()) {
      RCLCPP_ERROR(node_->get_logger(), "Service not registered: %s", service_name.c_str());
      stats_.errors++;
      return false;
    }
    client = it->second;
  }
  
  if (!client->is_ready()) {
    RCLCPP_WARN(node_->get_logger(), "Service not ready: %s", service_name.c_str());
    stats_.errors++;
    return false;
  }
  
  try {
    if (!client->call_async(srv_id, request)) {
      stats_.errors++;
      return false;
    }
    
    {
      std::lock_guard<std::mutex> lock(ros_clients_mutex_);
      pending_service_names_[srv_id] = service_name;
    }
    
    stats_.ros_services_called++;
    
    RCLCPP_DEBUG(node_->get_logger(), "Called ROS service: %s [ID: %u]",
                 service_name.c_str(), srv_id);
    
    return true;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to call service %s: %s",
                 service_name.c_str(), e.what());
    stats_.errors++;
    return false;
  }
}

void ServiceManager::process_pending_responses()
{
  std::lock_guard<std::mutex> lock(ros_clients_mutex_);
  
  std::vector<uint32_t> to_remove;
  
  for (auto& [srv_id, service_name] : pending_service_names_) {
    auto it = ros_service_clients_.find(service_name);
    if (it == ros_service_clients_.end()) continue;
    
    auto& client = it->second;
    
    std::vector<uint8_t> response;
    if (client->check_response(srv_id, response)) {
      // Response ready
      if (ros_response_callback_) {
        ros_response_callback_(srv_id, response);
      }
      stats_.responses_sent++;
      to_remove.push_back(srv_id);
    } else if (client->has_timedout(srv_id)) {
      // Timeout
      RCLCPP_WARN(node_->get_logger(), "Service call timeout [ID: %u]", srv_id);
      client->cleanup(srv_id);
      stats_.timeouts++;
      to_remove.push_back(srv_id);
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
    RCLCPP_WARN(node_->get_logger(), "Unity service already registered: %s", service_name.c_str());
    return true;
  }
  
  auto server = create_unity_service_server(service_name, service_type, request_callback);
  if (!server) {
    stats_.errors++;
    return false;
  }
  
  unity_service_servers_[service_name] = server;
  
  RCLCPP_INFO(node_->get_logger(), "Registered Unity service server: %s [%s]",
              service_name.c_str(), service_type.c_str());
  
  return true;
}

void ServiceManager::handle_unity_service_response(uint32_t srv_id,
                                                  const std::vector<uint8_t>& response)
{
  std::lock_guard<std::mutex> lock(unity_servers_mutex_);
  
  auto name_it = pending_unity_service_names_.find(srv_id);
  if (name_it == pending_unity_service_names_.end()) {
    RCLCPP_WARN(node_->get_logger(), "Received response for unknown Unity service ID: %u", srv_id);
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
  
  RCLCPP_ERROR(node_->get_logger(), "Failed to handle Unity service response [ID: %u]", srv_id);
}

} // namespace horus_unity_bridge
