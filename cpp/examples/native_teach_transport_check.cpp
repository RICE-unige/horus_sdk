#include "horus/teaching/runtime.hpp"
#include <chrono>
#include <iostream>
#include <thread>

// Test-only echo adapter: no controller, publisher, hardware or motion output.
struct InputFixture : horus::teaching::TeachAdapter {
  horus::teaching::AdapterCapabilities capabilities() const override {
    horus::teaching::AdapterCapabilities c;
    c.adapter_id = "input_fixture";
    c.qualification = "simulation";
    c.joint_names = {"fixture_joint"};
    c.command_frame = "fixture_base";
    c.calibration_id = "fixture-v1";
    c.controller_watchdog = "no-hardware-fixture";
    c.authority_contract = "fixture-session-lease";
    return c;
  }
  std::pair<bool, std::string> ready() override { return {true, ""}; }
  void anchor(const horus::teaching::Json &) override {}
  horus::teaching::Json apply(const horus::teaching::Json &sample) override {
    return {{"fixture_input", sample}, {"execution", "none"}};
  }
  bool hold(const horus::teaching::Json &) override { return true; }
};

int main(int argc, char **argv) {
  try {
    if (argc == 3 && std::string(argv[1]) == "--validate") {
      auto issues = horus::teaching::Episode(argv[2]).validate();
      std::cout << horus::teaching::Json(issues).dump() << '\n';
      return issues.empty() ? 0 : 1;
    }
    bool v2 = argc == 3 && std::string(argv[2]) == "--input-v2";
    if (argc != 2 && !v2)
      throw std::invalid_argument(
          "Provide a temporary recording directory [--input-v2]");
    horus::teaching::TeachConfig config(
        std::string(argv[1]), {{"minimum_free_bytes", 1024},
                               {"provenance", "fixture"},
                               {"streams",
                                {{{"stream_id", "state"},
                                  {"topic", "/horus_native_teach/state"},
                                  {"message_type", "std_msgs/msg/String"}},
                                 {{"stream_id", "image"},
                                  {"topic", "/horus_native_teach/image"},
                                  {"message_type", "sensor_msgs/msg/Image"},
                                  {"role", "camera"}},
                                 {{"stream_id", "info"}, {"topic", "/horus_native_teach/info"},
                                  {"message_type", "sensor_msgs/msg/CameraInfo"}, {"reliability", "reliable"}, {"durability", "transient_local"}},
                                 {{"stream_id", "static"}, {"topic", "/horus_native_teach/tf_static"},
                                  {"message_type", "tf2_msgs/msg/TFMessage"}, {"reliability", "reliable"}, {"durability", "transient_local"}}}}});
    if (v2)
      config = config.with_options(
          {{"input_schema_version", 2},
           {"required_input_features",
            {"head_pose", "left_stick", "right_stick", "left_buttons",
             "right_buttons"}},
           {"clutch_binding", "both_primary"},
           {"adapter_id", "input_fixture"},
           {"workflow", "collection"},
           {"joint_names", {"fixture_joint"}},
           {"joint_groups", {{"fixture", {"fixture_joint"}}}},
           {"command_frame", "fixture_base"},
           {"calibration_id", "fixture-v1"},
           {"grasp_bindings",
            {{"grasp", "left_grip"}, {"pinch", "right_trigger"}}}});
    horus::teaching::TeachRuntime runtime(config, "native_teach",
                                          v2 ? std::make_shared<InputFixture>()
                                             : nullptr);
    for (int i = 0; i < 140; ++i) {
      if (!runtime.status().at("running").get<bool>())
        throw std::runtime_error(runtime.status().dump());
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    std::cout << runtime.status().dump() << '\n';
    return 0;
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
