#include "registration_common.hpp"

int main() {
    auto set = examples::standard_fleet(
        {"atlas", "nova", "orion", "luna"},
        horus::core::RobotType::WHEELED,
        {0.8, 0.55, 0.45},
        "wheeled");
    examples::add_semantic_boxes(set.datavizs.front());
    return examples::register_or_fail(set);
}
