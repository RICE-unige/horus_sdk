#include "registration_common.hpp"

int main() {
    auto set = examples::standard_fleet(
        {"legged_1", "legged_2", "legged_3"},
        horus::core::RobotType::LEGGED,
        {0.65, 0.32, 0.45},
        "legged");
    return examples::register_or_fail(set);
}
