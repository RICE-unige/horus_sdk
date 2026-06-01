mod common;

fn main() {
    let mut set = common::robot_description_fleet();
    set.datavizs[0].add_3d_octomap(
        "/map_3d_octomap_mesh",
        "map",
        Some(common::options(&[
            ("render_mode", serde_json::json!("surface_mesh")),
            ("native_topic", serde_json::json!("/map_3d_octomap")),
        ])),
    );
    common::register_or_fail(&mut set);
}
