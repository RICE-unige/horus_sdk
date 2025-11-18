// Test publisher node for HORUS Unity Bridge
// Publishes fake sensor data for testing

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <chrono>

using namespace std::chrono_literals;

class TestPublisher : public rclcpp::Node
{
public:
  TestPublisher() : Node("test_publisher"), count_(0)
  {
    // Create publishers
    string_pub_ = create_publisher<std_msgs::msg::String>("/test/string", 10);
    twist_pub_ = create_publisher<geometry_msgs::msg::Twist>("/test/cmd_vel", 10);
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/test/pose", 10);
    scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>("/test/scan", 10);
    
    // Create timer to publish at 10 Hz
    timer_ = create_wall_timer(
      100ms,
      std::bind(&TestPublisher::publish_data, this)
    );
    
    RCLCPP_INFO(get_logger(), "Test publisher started - publishing on:");
    RCLCPP_INFO(get_logger(), "  /test/string (std_msgs/String)");
    RCLCPP_INFO(get_logger(), "  /test/cmd_vel (geometry_msgs/Twist)");
    RCLCPP_INFO(get_logger(), "  /test/pose (geometry_msgs/PoseStamped)");
    RCLCPP_INFO(get_logger(), "  /test/scan (sensor_msgs/LaserScan)");
  }

private:
  void publish_data()
  {
    count_++;
    
    // Publish string message
    auto string_msg = std_msgs::msg::String();
    string_msg.data = "Test message " + std::to_string(count_);
    string_pub_->publish(string_msg);
    
    // Publish twist message (robot velocity)
    auto twist_msg = geometry_msgs::msg::Twist();
    twist_msg.linear.x = 0.5 * std::sin(count_ * 0.1);
    twist_msg.angular.z = 0.3 * std::cos(count_ * 0.1);
    twist_pub_->publish(twist_msg);
    
    // Publish pose message
    auto pose_msg = geometry_msgs::msg::PoseStamped();
    pose_msg.header.stamp = now();
    pose_msg.header.frame_id = "map";
    pose_msg.pose.position.x = count_ * 0.01;
    pose_msg.pose.position.y = std::sin(count_ * 0.05);
    pose_msg.pose.position.z = 0.0;
    pose_msg.pose.orientation.w = 1.0;
    pose_pub_->publish(pose_msg);
    
    // Publish laser scan (every 10 messages)
    if (count_ % 10 == 0) {
      auto scan_msg = sensor_msgs::msg::LaserScan();
      scan_msg.header.stamp = now();
      scan_msg.header.frame_id = "laser";
      scan_msg.angle_min = -3.14159f;
      scan_msg.angle_max = 3.14159f;
      scan_msg.angle_increment = 0.01f;
      scan_msg.range_min = 0.1f;
      scan_msg.range_max = 10.0f;
      
      // Generate fake scan data
      int num_readings = static_cast<int>((scan_msg.angle_max - scan_msg.angle_min) / 
                                          scan_msg.angle_increment);
      scan_msg.ranges.resize(num_readings);
      for (int i = 0; i < num_readings; i++) {
        scan_msg.ranges[i] = 2.0f + std::sin(i * 0.01f) * 0.5f;
      }
      
      scan_pub_->publish(scan_msg);
    }
    
    if (count_ % 100 == 0) {
      RCLCPP_INFO(get_logger(), "Published %d messages", count_);
    }
  }
  
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr string_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  int count_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TestPublisher>();
  
  RCLCPP_INFO(node->get_logger(), "\n========================================");
  RCLCPP_INFO(node->get_logger(), "  Test Publisher for Unity Bridge");
  RCLCPP_INFO(node->get_logger(), "========================================");
  RCLCPP_INFO(node->get_logger(), "Publishing test messages at 10 Hz");
  RCLCPP_INFO(node->get_logger(), "Press Ctrl+C to stop\n");
  
  rclcpp::spin(node);
  rclcpp::shutdown();
  
  return 0;
}
