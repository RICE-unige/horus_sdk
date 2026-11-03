#include <horus/bridge/robot_registry.hpp>
#include <horus/mapping/costmap.hpp>
#include <horus/mapping/mesh_protocol.hpp>
#include <horus/mapping/live_mesh.hpp>
#include <horus/mapping/live_mesh_publisher.hpp>
#include <horus/mapping/mesh_page_scheduler.hpp>
#include <horus/client.hpp>
#include <horus/description/compiler.hpp>
#include <horus/description/resolver.hpp>
#include <horus/teaching/session.hpp>
#include <horus/mapping/costmap_adapter.hpp>
#include <horus/mapping/mesh_source.hpp>
#include <horus/mapping/mesh_source_adapter.hpp>
#include <horus/monitoring/dashboard.hpp>
#include <horus/remote_rendering/protocol.hpp>
#include <horus/remote_rendering/gpu.hpp>
#include <horus/remote_rendering/frame.hpp>
#include <horus/remote_rendering/stereo.hpp>
#include <horus/remote_rendering/encoder.hpp>
#include <horus/remote_rendering/publisher.hpp>
#include <horus/remote_rendering/scene.hpp>
#include <horus/core/topic_map.hpp>
#include <horus/tf.hpp>
#include <horus/remote_rendering/gaussian.hpp>
#include <horus/remote_rendering/video.hpp>
#include <horus/remote_rendering/atlas.hpp>
#include <horus/experiments/workloads.hpp>
#include <horus/dataviz/collision_risk.hpp>
#include <stdexcept>

int main() {
    if(horus::remote_rendering::scene_catalog().empty()) return 1;
    const auto workload=horus::experiments::workload_from_dict({{"camera",{{"streams",2}}}});
    if(workload.stream_count()!=2) return 1;
    if(horus::dataviz::risk_from_distance(.5,1.)!=.5) return 1;
    const std::array<horus::remote_rendering::RemoteRenderViewSpec,1> views{};
    const auto atlas=horus::remote_rendering::render_multiview_atlas({}, {}, views,{.width=16,.height=16});
    if(atlas.rgb.size()!=16*16*3) return 1;
    (void)horus::remote_rendering::H264Encoder::compiled();
    try { horus::remote_rendering::load_scene("/missing/horus-scene.ply"); return 1; }
    catch(const std::invalid_argument&) {}
    const auto depth_codes = horus::remote_rendering::quantize_depth(std::array<float, 2>{.1f,10.f},.1f,10.f);
    if (depth_codes != std::vector<uint16_t>{1,65535}) return 1;
    horus::mapping::MeshSourceState source;
    if(source.state().page_count()!=0) return 1;
    const auto projection=horus::remote_rendering::eye_fov_from_unity_projection(1,1,0,0);
    if(projection.left!=-1) return 1;
    if(!horus::monitoring::is_bridge_node("/team/horus_unity_bridge")) return 1;
    try { horus::remote_rendering::CudaRenderer gpu("/missing/horus.so",1); return 1; }
    catch(const std::runtime_error&) {}
    const auto model = horus::description::compile_urdf(
        "<robot name='installed'><link name='base'/></robot>", nlohmann::json::object());
    if (model.manifest.at("link_count") != 1 || model.encoded_payload.empty()) return 1;
    horus::teaching::TeachConfig config("/tmp/horus-installed-consumer");
    horus::teaching::TeachSession session(config, "robot");
    if (session.state().at("control_ready") != false) return 1;
    session.close();
    horus::mapping::CostmapState grid;
    grid.full({"map", 2, 2, 0.05}, {0, 1, 99, 100}, 1);
    grid.patch("map", 0, 0, 1, 1, std::array<std::int8_t,1>{-1}, 2);
    if (grid.data()[0] != -1) throw std::runtime_error("Installed map API failed");
    auto clear = horus::mapping::encode_live_payload(0, {0,0,0}, 1, 0,
        horus::mapping::MeshOperation::Clear, nullptr, 52);
    if (horus::mapping::decode_live_header(clear).epoch != 1) return 1;
    horus::mapping::LiveMeshState state({.initial_epoch=1});
    if (state.clear()->epoch != 2) return 1;
    horus::mapping::MeshPageScheduler scheduler;
    scheduler.enqueue(state.snapshot_clear_update());
    if (!scheduler.pop_ready(1)) return 1;
#if defined(HORUS_ROS2_ENABLED)
    horus::dataviz::CollisionRiskAnalyzer collision({.robot_name="consumer",.source_topic="/installed_consumer/scan"});
    collision.stop();
    horus::tf::Publisher tf({.topic="/installed_consumer/tf", .static_topic="/installed_consumer/tf_static"});
    tf.publish({});
    (void)tf.subscriber_count();
    horus::remote_rendering::RemotePublisher remote_publisher;
    if(remote_publisher.status().at("state")!="waiting_for_view") return 1;
    remote_publisher.stop();
    horus::mapping::LiveMeshPublisher publisher({.topic="/installed_consumer_mesh"});
    if (publisher.pump() != 0) return 1;
#endif
    horus::Client client("ros2",false);
    client.shutdown();
    horus::robot::Robot robot("consumer", horus::core::RobotType::WHEELED);
    horus::bridge::RobotRegistryClient registry;
    auto payload = registry.build_robot_config_dict(robot, *robot.create_dataviz());
    if (horus::bridge::registration_json(payload).find("consumer") == std::string::npos) return 1;
    auto workspace = horus::robot::make_workspace_visualization("map_view");
    auto view = workspace.create_dataviz();
    view->add_remote_rendered_map("/remote/map", "world");
    const auto remote = registry.build_robot_config_dict(workspace, *view);
    if (remote.entity_kind != "workspace_visualization" || remote.control.teleop.enabled ||
        remote.global_visualizations.size() != 1 || remote.global_visualizations[0].remote_render.empty()) return 1;
    registry.stop();
}
