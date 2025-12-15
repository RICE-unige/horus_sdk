/**
 * @file sdk_registration_demo.cpp
 * @brief Example of registering a custom robot with HORUS using the C++ SDK
 * 
 * Usage:
 *   mkdir build && cd build
 *   cmake ..
 *   make
 *   ./sdk_registration_demo
 */

#include <horus/robot/robot.hpp>
#include <horus/robot/sensors.hpp>
#include <horus/core/event_bus.hpp>
#include <horus/utils/branding.hpp>
#include <iostream>
#include <memory>
#include <thread>
#include <chrono>

int main() {
    // Display HORUS branding
    horus::utils::show_ascii_art();
    
    std::cout << "\n📋 Defining Robot Configuration...\n" << std::endl;
    
    // 1. Define your robot
    horus::robot::Robot my_robot("SdkBot_Professional", horus::core::RobotType::WHEELED);
    
    std::cout << "✅ Created robot: " << my_robot.get_name() << std::endl;
    
    // 2. Add Sensors
    auto front_laser = std::make_shared<horus::robot::LaserScan>(
        "Front Lidar",
        "laser_frame",
        "/scan",
        -3.14159f,  // min_angle
        3.14159f,   // max_angle
        0.005f,     // angle_increment
        0.1f,       // min_range
        12.0f,      // max_range
        0.01f,      // range_resolution
        "#00FFFF",  // color (cyan)
        0.1f        // point_size
    );
    
    my_robot.add_sensor(front_laser);
    std::cout << "✅ Added sensor: Front Lidar" << std::endl;
    
    auto camera = std::make_shared<horus::robot::Camera>(
        "Realsense RGB",
        "camera_link",
        "/camera/color/image_raw"
    );
    
    my_robot.add_sensor(camera);
    std::cout << "✅ Added sensor: Realsense RGB Camera" << std::endl;
    
    // 3. Add metadata
    my_robot.add_metadata("description", std::string("Professional wheeled robot"));
    my_robot.add_metadata("max_speed", 2.5);
    
    std::cout << "\n📊 Robot Configuration Summary:" << std::endl;
    std::cout << "  Name: " << my_robot.get_name() << std::endl;
    std::cout << "  Type: " << my_robot.get_type_str() << std::endl;
    std::cout << "  Sensors: " << my_robot.get_sensor_count() << std::endl;
    
    // 4. Subscribe to events
    std::cout << "\n📡 Setting up event subscriptions..." << std::endl;
    
    auto sub_id = horus::core::subscribe("robot.*", 
        [](const horus::core::Event& event) {
            std::cout << "📬 Received event: " << event.topic << std::endl;
        }, 
        horus::core::EventPriority::LOW
    );
    
    std::cout << "✅ Subscribed to robot events" << std::endl;
    
    // 5. Publish a status event
    std::cout << "\n📤 Publishing robot status event..." << std::endl;
    
    horus::core::publish("robot.status.ready", 
                        std::any{},
                        horus::core::EventPriority::NORMAL,
                        "demo");
    
    // Give the event system time to process
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // 6. Registration with HORUS backend would happen here
    std::cout << "\n🔗 Registration with HORUS backend:" << std::endl;
    std::cout << "  Note: Full HORUS backend integration requires ROS 2 runtime" << std::endl;
    std::cout << "  This demo shows the robot configuration setup" << std::endl;
    
    // Show event bus statistics
    auto stats = horus::core::get_event_bus().get_stats();
    std::cout << "\n📈 Event Bus Statistics:" << std::endl;
    std::cout << "  Events Published: " << stats["events_published"] << std::endl;
    std::cout << "  Events Processed: " << stats["events_processed"] << std::endl;
    std::cout << "  Active Subscriptions: " << stats["active_subscriptions"] << std::endl;
    
    std::cout << "\n✨ Demo completed successfully!" << std::endl;
    
    // Cleanup
    horus::core::unsubscribe(sub_id);
    
    return 0;
}
