#include "horus/bridge/robot_registry.hpp"
#include "horus/description/compiler.hpp"
#include "horus/description/resolver.hpp"
#include <chrono>
#include <iostream>
#include <thread>
int main(int argc, char **argv) {
    try {
        if (argc < 2)
            return 1;
        if (std::string(argv[1]) == "--source") {
            if (argc < 3)
                return 1;
            std::cout
                << horus::description::resolve(nlohmann::json::parse(argv[2])).to_dict().dump()
                << '\n';
            return 0;
        }
        if (argc > 2 && std::string(argv[2]) == "--artifact") {
            std::cout << horus::description::compile_file({{"urdf_path", argv[1]},
                                                           {"base_frame", "base"},
                                                           {"body_mesh_mode", "runtime_high_mesh"},
                                                           {"visual_link_pose_source", "tf"}})
                             .to_dict()
                             .dump()
                      << '\n';
            return 0;
        }
        horus::robot::Robot robot("native_model", horus::core::RobotType::WHEELED);
        horus::robot::RobotDescriptionOptions config;
        config.urdf_path = argv[1];
        config.base_frame = "base";
        config.body_mesh_mode = "runtime_high_mesh";
        config.chunk_size_bytes = 1024;
        config.visual_link_pose_source = "tf";
        robot.configure_robot_description(config);
        horus::bridge::RegistryOptions options;
        options.auto_launch = false;
        horus::bridge::RobotRegistryClient client(options);
        auto viz = robot.create_dataviz();
        auto result = client.register_robot(robot, *viz, 8, false, false);
        if (!result.first) {
            std::cerr << std::any_cast<std::string>(result.second.at("error")) << '\n';
            return 2;
        }
        std::this_thread::sleep_for(std::chrono::seconds(10));
        return 0;
    } catch (const std::exception &e) {
        std::cerr << e.what() << '\n';
        return 3;
    }
}
