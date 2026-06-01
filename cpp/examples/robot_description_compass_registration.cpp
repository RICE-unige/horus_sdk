#include "registration_common.hpp"

int main() {
    auto set = examples::robot_description_fleet();
    for (auto& robot : set.robots) {
        robot.configure_workspace_compass(true, 8088, "auto");
    }
    return examples::register_or_fail(set);
}
