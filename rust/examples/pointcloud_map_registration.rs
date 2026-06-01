mod common;

fn main() {
    let mut set = common::robot_description_fleet();
    set.datavizs[0].add_3d_map(
        "/map_3d",
        "map",
        Some(common::options(&[
            ("point_size", serde_json::json!(0.035)),
            ("render_all_points", serde_json::json!(true)),
            ("point_shape", serde_json::json!("circle")),
        ])),
    );
    common::register_or_fail(&mut set);
}
