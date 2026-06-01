mod common;

fn main() {
    let mut set = common::robot_description_fleet();
    for robot in &mut set.robots {
        robot.configure_workspace_compass(true, 8088, "auto");
    }
    common::register_or_fail(&mut set);
}
