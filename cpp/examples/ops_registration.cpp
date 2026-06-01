#include "registration_common.hpp"

int main() {
    auto set = examples::standard_fleet(
        {"atlas", "nova", "orion"},
        horus::core::RobotType::WHEELED,
        {0.8, 0.55, 0.45},
        "wheeled");
    return examples::register_or_fail(set);
}
