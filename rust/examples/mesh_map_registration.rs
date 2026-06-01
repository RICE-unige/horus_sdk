mod common;

fn main() {
    let mut set = common::robot_description_fleet();
    set.datavizs[0].add_3d_mesh(
        "/map_3d_mesh",
        "map",
        Some(common::options(&[
            ("max_triangles", serde_json::json!(60000)),
            ("use_vertex_colors", serde_json::json!(true)),
        ])),
    );
    common::register_or_fail(&mut set);
}
