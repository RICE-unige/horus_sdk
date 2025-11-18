#include <rclcpp/rclcpp.hpp>
#include <example_interfaces/srv/add_two_ints.hpp>
#include <chrono>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("test_service_client");

  auto client = node->create_client<example_interfaces::srv::AddTwoInts>("/unity/add_two_ints");

  if (!client->wait_for_service(std::chrono::seconds(2))) {
    RCLCPP_ERROR(node->get_logger(), "Unity service not available");
    return 1;
  }

  auto req = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
  req->a = 10;
  req->b = 20;

  auto future = client->async_send_request(req);
  auto ret = rclcpp::spin_until_future_complete(node, future);
  if (ret == rclcpp::FutureReturnCode::SUCCESS) {
    auto resp = future.get();
    RCLCPP_INFO(node->get_logger(), "Unity AddTwoInts response: %ld", resp->sum);
  } else {
    RCLCPP_ERROR(node->get_logger(), "Unity service call failed");
  }

  rclcpp::shutdown();
  return 0;
}