#include <rclcpp/rclcpp.hpp>
#include <example_interfaces/srv/add_two_ints.hpp>

class TestServiceServer : public rclcpp::Node {
public:
  TestServiceServer() : Node("test_service_server") {
    service_ = create_service<example_interfaces::srv::AddTwoInts>(
      "/test/add_two_ints",
      std::bind(&TestServiceServer::handle, this, std::placeholders::_1, std::placeholders::_2)
    );
    RCLCPP_INFO(get_logger(), "Test service server ready: /test/add_two_ints");
  }

private:
  void handle(const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
              std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response) {
    response->sum = request->a + request->b;
    RCLCPP_INFO(get_logger(), "AddTwoInts: %ld + %ld = %ld", request->a, request->b, response->sum);
  }

  rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr service_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TestServiceServer>());
  rclcpp::shutdown();
  return 0;
}
