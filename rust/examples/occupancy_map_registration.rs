mod common;

fn main() {
    let mut set = common::robot_description_fleet();
    set.datavizs[0].add_occupancy_grid(
        "/map",
        "map",
        Some(common::options(&[
            ("color_free", serde_json::json!("#222222")),
            ("color_occupied", serde_json::json!("#F4F4F4")),
            ("alpha", serde_json::json!(0.85)),
        ])),
    );
    common::register_or_fail(&mut set);
}
