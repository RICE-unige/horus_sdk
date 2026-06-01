mod common;

fn main() {
    let mut set = common::robot_description_fleet();
    common::register_or_fail(&mut set);
}
