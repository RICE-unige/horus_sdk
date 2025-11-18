// Unity Client Simulator for HORUS Unity Bridge testing
// Simulates Unity TCP client behavior

#include <iostream>
#include <string>
#include <vector>
#include <cstring>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <chrono>
#include <thread>

class UnityClientSimulator
{
public:
  UnityClientSimulator(const std::string& host, int port)
    : host_(host), port_(port), sock_(-1), connected_(false)
  {
  }
  
  ~UnityClientSimulator()
  {
    disconnect();
  }
  
  bool connect()
  {
    sock_ = socket(AF_INET, SOCK_STREAM, 0);
    if (sock_ < 0) {
      std::cerr << "Failed to create socket" << std::endl;
      return false;
    }
    
    struct sockaddr_in server_addr;
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(port_);
    
    if (inet_pton(AF_INET, host_.c_str(), &server_addr.sin_addr) <= 0) {
      std::cerr << "Invalid address: " << host_ << std::endl;
      return false;
    }
    
    if (::connect(sock_, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
      std::cerr << "Connection failed to " << host_ << ":" << port_ << std::endl;
      return false;
    }
    
    connected_ = true;
    std::cout << "Connected to Unity bridge at " << host_ << ":" << port_ << std::endl;
    return true;
  }
  
  void disconnect()
  {
    if (sock_ >= 0) {
      close(sock_);
      sock_ = -1;
    }
    connected_ = false;
  }
  
  bool send_subscribe_command(const std::string& topic, const std::string& message_type)
  {
    std::string params = "{\"topic\":\"" + topic + "\",\"message_name\":\"" + message_type + "\"}";
    return send_system_command("__subscribe", params);
  }
  
  bool send_publish_command(const std::string& topic, const std::string& message_type)
  {
    std::string params = "{\"topic\":\"" + topic + "\",\"message_name\":\"" + message_type + "\"}";
    return send_system_command("__publish", params);
  }
  
  bool send_handshake()
  {
    std::string params = "{}";
    return send_system_command("__handshake", params);
  }
  
  bool send_topic_list_request()
  {
    std::string params = "{}";
    return send_system_command("__topic_list", params);
  }
  
  bool send_ros_service_register(const std::string& name, const std::string& srv_type)
  {
    // Register ROS service client we can call
    std::string params = "{\"topic\":\"" + name + "\",\"message_name\":\"" + srv_type + "\"}";
    return send_system_command("__ros_service", params);
  }
  
  bool send_unity_service_register(const std::string& name, const std::string& srv_type)
  {
    // Register a Unity-implemented service that ROS can call
    std::string params = "{\"topic\":\"" + name + "\",\"message_name\":\"" + srv_type + "\"}";
    return send_system_command("__unity_service", params);
  }
  
  bool send_service_request(uint32_t srv_id, const std::string& service_name, const std::vector<uint8_t>& request)
  {
    // Send __request with srv_id, then payload with destination=service_name
    std::string params = std::string("{\"srv_id\":") + std::to_string(srv_id) + "}";
    if (!send_system_command("__request", params)) return false;
    return send_message(service_name, request);
  }
  
  bool send_service_response(uint32_t srv_id, const std::vector<uint8_t>& response)
  {
    // Send __response with srv_id, then payload to dummy destination
    std::string params = std::string("{\"srv_id\":") + std::to_string(srv_id) + "}";
    if (!send_system_command("__response", params)) return false;
    return send_message("__response_payload", response);
  }
  
  bool send_keepalive()
  {
    return send_message("", std::vector<uint8_t>());
  }
  
  void receive_loop(int duration_sec)
  {
    std::cout << "Receiving messages for " << duration_sec << " seconds..." << std::endl;
    
    auto start = std::chrono::steady_clock::now();
    int message_count = 0;
    
    while (connected_) {
      // Check timeout
      auto now = std::chrono::steady_clock::now();
      auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - start).count();
      if (elapsed >= duration_sec) {
        break;
      }
      
      // Try to receive a message
      std::string destination;
      std::vector<uint8_t> payload;
      
      if (receive_message(destination, payload)) {
        message_count++;
        
        if (destination == "__request") {
          // Parse srv_id
          std::string json(payload.begin(), payload.end());
          uint32_t srv_id = 0;
          auto pos = json.find("srv_id");
          if (pos != std::string::npos) {
            auto colon = json.find(":", pos);
            if (colon != std::string::npos) {
              srv_id = static_cast<uint32_t>(std::stoul(json.substr(colon+1)));
            }
          }
          // Next message is the request payload with service name destination
          std::string service_dest;
          std::vector<uint8_t> req_payload;
          if (receive_message(service_dest, req_payload)) {
            // Decode AddTwoInts request if matches service
            if (service_dest == "/unity/add_two_ints" || service_dest == "/test/add_two_ints") {
              auto decode_int64 = [](const std::vector<uint8_t>& d, size_t off){
                int64_t v=0; for(int i=7;i>=0;--i){ v = (v<<8) | d[off+i]; } return v; };
              if (req_payload.size() >= 16) {
                int64_t a = decode_int64(req_payload, 0);
                int64_t b = decode_int64(req_payload, 8);
                int64_t sum = a + b;
                // Encode response: one int64
                std::vector<uint8_t> resp(8);
                for (int i=0;i<8;++i) resp[i] = (uint8_t)(((uint64_t)sum) >> (8*i));
                send_service_response(srv_id, resp);
                std::cout << "  Unity service handled: " << service_dest << " sum=" << sum << std::endl;
              }
            }
          }
        } else if (!destination.empty() && destination[0] == '_') {
          std::cout << "  System message: " << destination << " (" << payload.size() << " bytes)" << std::endl;
        } else if (!destination.empty()) {
          std::cout << "  Data message: " << destination << " (" << payload.size() << " bytes)" << std::endl;
        }
      }
      
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    
    std::cout << "Received " << message_count << " messages in " << duration_sec << " seconds" << std::endl;
  }

private:
  bool send_system_command(const std::string& command, const std::string& params)
  {
    // System commands are sent with command as destination and JSON params as payload
    std::vector<uint8_t> payload(params.begin(), params.end());
    return send_message(command, payload);
  }
  
  bool send_message(const std::string& destination, const std::vector<uint8_t>& payload)
  {
    if (!connected_) {
      return false;
    }
    
    std::vector<uint8_t> message;
    
    // Destination length (4 bytes, little-endian)
    uint32_t dest_len = destination.size();
    message.push_back(dest_len & 0xFF);
    message.push_back((dest_len >> 8) & 0xFF);
    message.push_back((dest_len >> 16) & 0xFF);
    message.push_back((dest_len >> 24) & 0xFF);
    
    // Destination string
    message.insert(message.end(), destination.begin(), destination.end());
    
    // Payload length (4 bytes, little-endian)
    uint32_t payload_len = payload.size();
    message.push_back(payload_len & 0xFF);
    message.push_back((payload_len >> 8) & 0xFF);
    message.push_back((payload_len >> 16) & 0xFF);
    message.push_back((payload_len >> 24) & 0xFF);
    
    // Payload data
    message.insert(message.end(), payload.begin(), payload.end());
    
    // Send
    ssize_t sent = send(sock_, message.data(), message.size(), 0);
    if (sent < 0) {
      std::cerr << "Send failed" << std::endl;
      return false;
    }
    
    return true;
  }
  
  bool receive_message(std::string& destination, std::vector<uint8_t>& payload)
  {
    // Read destination length
    uint32_t dest_len;
    if (!read_uint32(dest_len)) {
      return false;
    }
    
    // Read destination
    destination.resize(dest_len);
    if (dest_len > 0) {
      if (!read_bytes(reinterpret_cast<uint8_t*>(&destination[0]), dest_len)) {
        return false;
      }
    }
    
    // Read payload length
    uint32_t payload_len;
    if (!read_uint32(payload_len)) {
      return false;
    }
    
    // Read payload
    payload.resize(payload_len);
    if (payload_len > 0) {
      if (!read_bytes(payload.data(), payload_len)) {
        return false;
      }
    }
    
    return true;
  }
  
  bool read_uint32(uint32_t& value)
  {
    uint8_t bytes[4];
    if (!read_bytes(bytes, 4)) {
      return false;
    }
    value = static_cast<uint32_t>(bytes[0]) |
            (static_cast<uint32_t>(bytes[1]) << 8) |
            (static_cast<uint32_t>(bytes[2]) << 16) |
            (static_cast<uint32_t>(bytes[3]) << 24);
    return true;
  }
  
  bool read_bytes(uint8_t* buffer, size_t size)
  {
    size_t total_read = 0;
    while (total_read < size) {
      ssize_t n = recv(sock_, buffer + total_read, size - total_read, 0);
      if (n <= 0) {
        if (n == 0 || (errno != EAGAIN && errno != EWOULDBLOCK)) {
          connected_ = false;
        }
        return false;
      }
      total_read += n;
    }
    return true;
  }
  
  std::string host_;
  int port_;
  int sock_;
  bool connected_;
};

int main(int argc, char** argv)
{
  std::string host = argc > 1 ? argv[1] : "127.0.0.1";
  int port = argc > 2 ? std::atoi(argv[2]) : 10000;
  
  std::cout << "\n========================================" << std::endl;
  std::cout << "  Unity Client Simulator" << std::endl;
  std::cout << "========================================" << std::endl;
  std::cout << "Connecting to: " << host << ":" << port << std::endl;
  std::cout << "========================================\n" << std::endl;
  
  UnityClientSimulator client(host, port);
  
  if (!client.connect()) {
    std::cerr << "Failed to connect to bridge" << std::endl;
    return 1;
  }
  
  // Test sequence
  std::cout << "\n1. Sending handshake..." << std::endl;
  client.send_handshake();
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  
  std::cout << "\n2. Requesting topic list..." << std::endl;
  client.send_topic_list_request();
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  
  std::cout << "\n3. Subscribing to /test/string..." << std::endl;
  client.send_subscribe_command("/test/string", "std_msgs/String");
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  
  std::cout << "\n4. Subscribing to /test/pose..." << std::endl;
  client.send_subscribe_command("/test/pose", "geometry_msgs/PoseStamped");
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  
  std::cout << "\n5. Registering publisher for /test/cmd_vel..." << std::endl;
  client.send_publish_command("/test/cmd_vel", "geometry_msgs/Twist");
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  
  std::cout << "\n6. Receiving messages..." << std::endl;
  client.receive_loop(5);

  // Service tests
  std::cout << "\n7. Registering Unity service at /unity/add_two_ints..." << std::endl;
  client.send_unity_service_register("/unity/add_two_ints", "example_interfaces/srv/AddTwoInts");
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  std::cout << "\n8. Registering ROS service client for /test/add_two_ints..." << std::endl;
  client.send_ros_service_register("/test/add_two_ints", "example_interfaces/srv/AddTwoInts");
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  std::cout << "\n9. Calling ROS service /test/add_two_ints (1+2)..." << std::endl;
  // Build serialized AddTwoInts request manually: two int64 fields (little-endian)
  auto encode_int64 = [](int64_t v) {
    std::vector<uint8_t> b(8);
    for (int i=0;i<8;++i) b[i] = (uint8_t)((uint64_t)v >> (8*i));
    return b;
  };
  std::vector<uint8_t> req;
  auto a = encode_int64(1);
  auto b = encode_int64(2);
  req.insert(req.end(), a.begin(), a.end());
  req.insert(req.end(), b.begin(), b.end());
  uint32_t srv_id = 42;
  client.send_service_request(srv_id, "/test/add_two_ints", req);
  
  // Wait to receive response (not fully parsed here, just ensure flow works)
  client.receive_loop(5);
  
  std::cout << "\nTest completed successfully!" << std::endl;
  std::cout << "========================================\n" << std::endl;
  
  client.disconnect();
  return 0;
}
