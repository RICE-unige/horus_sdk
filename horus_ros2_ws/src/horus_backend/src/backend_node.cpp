#include "horus_backend/backend_node.hpp"
#include <iostream>

namespace horus_backend {

BackendNode::BackendNode() : Node("horus_backend_node") {
    setup_parameters();
    setup_publishers();
    setup_subscribers();
    setup_services();
}

BackendNode::~BackendNode() {
    shutdown();
}

void BackendNode::initialize() {
    // Initialize plugin manager
    plugin_manager_ = std::make_unique<PluginManager>();
    plugin_manager_->load_plugins();
    
    // Initialize TCP server
    tcp_server_ = std::make_unique<TcpServer>(tcp_port_);
    tcp_server_->set_message_callback([this](const std::string& message) {
        return process_message(message);
    });
    
    // Start TCP server
    tcp_server_->start();
    
    display_startup_info();
}

void BackendNode::shutdown() {
    RCLCPP_INFO(get_logger(), "Shutting down HORUS Backend...");
    
    if (tcp_server_) {
        tcp_server_->stop();
    }
    
    RCLCPP_INFO(get_logger(), "HORUS Backend shutdown complete");
}

void BackendNode::setup_parameters() {
    // Declare parameters with defaults
    declare_parameter<int>("tcp_port", 8080);
    declare_parameter<std::string>("log_level", "info");
    
    // Get parameter values
    tcp_port_ = get_parameter("tcp_port").as_int();
    log_level_ = get_parameter("log_level").as_string();
}

void BackendNode::setup_publishers() {
    command_publisher_ = create_publisher<horus_interfaces::msg::RobotCommand>(
        "horus/robot_commands", 10);
    
    status_publisher_ = create_publisher<horus_interfaces::msg::RobotStatus>(
        "horus/robot_status", 10);
}

void BackendNode::setup_subscribers() {
    status_subscriber_ = create_subscription<horus_interfaces::msg::RobotStatus>(
        "horus/robot_status_feedback", 10,
        std::bind(&BackendNode::status_callback, this, std::placeholders::_1));
}

void BackendNode::setup_services() {
    connect_service_ = create_service<horus_interfaces::srv::ConnectRobot>(
        "horus/connect_robot",
        std::bind(&BackendNode::connect_robot_callback, this, 
                  std::placeholders::_1, std::placeholders::_2));
    
    execute_service_ = create_service<horus_interfaces::srv::ExecuteCommand>(
        "horus/execute_command",
        std::bind(&BackendNode::execute_command_callback, this,
                  std::placeholders::_1, std::placeholders::_2));
}

void BackendNode::status_callback(const horus_interfaces::msg::RobotStatus::SharedPtr msg) {
    RCLCPP_DEBUG(get_logger(), "Received status from robot: %s", msg->robot_id.c_str());
    // Forward status to connected SDK clients via TCP
    // This will be implemented when we add the TCP message processing
}

void BackendNode::connect_robot_callback(
    const std::shared_ptr<horus_interfaces::srv::ConnectRobot::Request> request,
    std::shared_ptr<horus_interfaces::srv::ConnectRobot::Response> response) {
    
    RCLCPP_INFO(get_logger(), "Connect robot request: %s (type: %s)", 
                request->robot_id.c_str(), request->robot_type.c_str());
    
    bool success = plugin_manager_->connect_robot(request->robot_id, request->robot_type);
    response->success = success;
    response->message = success ? "Robot connected successfully" : "Failed to connect robot";
}

void BackendNode::execute_command_callback(
    const std::shared_ptr<horus_interfaces::srv::ExecuteCommand::Request> request,
    std::shared_ptr<horus_interfaces::srv::ExecuteCommand::Response> response) {
    
    RCLCPP_INFO(get_logger(), "Execute command for robot: %s, command: %s", 
                request->robot_id.c_str(), request->command_type.c_str());
    
    // For now, just acknowledge the command
    response->success = true;
    response->result_data = "Command executed";
    response->message = "Command processed successfully";
}

void BackendNode::display_startup_info() {
    RCLCPP_INFO(get_logger(), "========================================");
    RCLCPP_INFO(get_logger(), "    HORUS ROS2 Backend Node");
    RCLCPP_INFO(get_logger(), "========================================");
    RCLCPP_INFO(get_logger(), "Version: 0.1.0");
    RCLCPP_INFO(get_logger(), "TCP Port: %d", tcp_port_);
    RCLCPP_INFO(get_logger(), "Log Level: %s", log_level_.c_str());
    RCLCPP_INFO(get_logger(), "Available Plugins: %zu", plugin_manager_->get_available_plugins().size());
    RCLCPP_INFO(get_logger(), "========================================");
    RCLCPP_INFO(get_logger(), "Backend ready for SDK connections");
}

std::string BackendNode::process_message(const std::string& message) {
    // Basic message processing - will be expanded later
    RCLCPP_DEBUG(get_logger(), "Received TCP message: %s", message.c_str());
    
    // For now, just echo back a simple response
    return "{\"status\": \"ok\", \"message\": \"Backend received: " + message + "\"}";
}

} // namespace horus_backend