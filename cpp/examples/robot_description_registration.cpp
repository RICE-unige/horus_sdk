#include "registration_common.hpp"

int main() {
    auto set = examples::robot_description_fleet();
    return examples::register_or_fail(set);
}
