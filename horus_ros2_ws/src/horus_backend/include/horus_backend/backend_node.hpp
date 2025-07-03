#pragma once

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <string>

#include "horus_backend/tcp_server.hpp"
#include "horus_backend/plugin_manager.hpp"
#include "horus_interfaces/msg/robot_command.hpp"
#include "horus_interfaces/msg/robot_status.hpp"
#include "horus_interfaces/srv/connect_robot.hpp"
#include "horus_interfaces/srv/execute_command.hpp"

namespace horus_backend {

class BackendNode : public rclcpp::Node {
public:
    BackendNode();
    ~BackendNode();
    
    void initialize();
    void shutdown();

private:
    // Core components
    std::unique_ptr<TcpServer> tcp_server_;
    std::unique_ptr<PluginManager> plugin_manager_;
    
    // ROS2 Publishers
    rclcpp::Publisher<horus_interfaces::msg::RobotCommand>::SharedPtr command_publisher_;
    rclcpp::Publisher<horus_interfaces::msg::RobotStatus>::SharedPtr status_publisher_;
    
    // ROS2 Subscribers
    rclcpp::Subscription<horus_interfaces::msg::RobotStatus>::SharedPtr status_subscriber_;
    
    // ROS2 Services
    rclcpp::Service<horus_interfaces::srv::ConnectRobot>::SharedPtr connect_service_;
    rclcpp::Service<horus_interfaces::srv::ExecuteCommand>::SharedPtr execute_service_;
    
    // Parameters
    int tcp_port_;
    std::string log_level_;
    
    // Callbacks
    void status_callback(const horus_interfaces::msg::RobotStatus::SharedPtr msg);
    void connect_robot_callback(
        const std::shared_ptr<horus_interfaces::srv::ConnectRobot::Request> request,
        std::shared_ptr<horus_interfaces::srv::ConnectRobot::Response> response);
    void execute_command_callback(
        const std::shared_ptr<horus_interfaces::srv::ExecuteCommand::Request> request,
        std::shared_ptr<horus_interfaces::srv::ExecuteCommand::Response> response);
    
    // Internal methods
    void setup_parameters();
    void setup_publishers();
    void setup_subscribers();
    void setup_services();
    void display_startup_info();
    std::string process_message(const std::string& message);
};

} // namespace horus_backend