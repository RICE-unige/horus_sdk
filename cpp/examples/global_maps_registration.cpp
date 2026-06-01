#include "registration_common.hpp"

int main() {
    auto set = examples::standard_fleet(
        {"test_bot_1", "test_bot_2"},
        horus::core::RobotType::WHEELED,
        {0.8, 0.55, 0.45},
        "wheeled");
    examples::add_world_maps(set.datavizs.front());
    return examples::register_or_fail(set);
}
