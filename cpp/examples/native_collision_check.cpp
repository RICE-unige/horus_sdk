#include "horus/dataviz/collision_risk.hpp"
#include <chrono>
#include <thread>
int main(int argc, char **argv) {
    if (argc != 2)
        return 2;
    horus::dataviz::CollisionRiskOptions o;
    o.robot_name = "parity";
    o.source_type = argv[1];
    o.source_topic = "/parity/source";
    o.output_topic = "/parity/risk";
    o.smoothing_alpha = 1;
    horus::dataviz::CollisionRiskAnalyzer analyzer(o);
    std::this_thread::sleep_for(std::chrono::seconds(7));
    analyzer.stop();
    analyzer.stop();
}
