#include "horus/remote_rendering/encoder.hpp"
#include <cassert>

int main() {
    using namespace horus::remote_rendering;
    for (const auto options : {H264EncoderOptions{0, 0}, H264EncoderOptions{383, 64},
                               H264EncoderOptions{384, 64, 0}, H264EncoderOptions{8192, 8192}}) {
        bool rejected = false;
        try {
            H264Encoder encoder(options);
        } catch (const std::invalid_argument &) {
            rejected = true;
        }
        assert(rejected);
        rejected = false;
        try {
            AsyncH264Encoder encoder(options);
        } catch (const std::invalid_argument &) {
            rejected = true;
        }
        assert(rejected);
    }
    if (!H264Encoder::compiled()) {
        bool rejected = false;
        try {
            H264Encoder encoder({384, 64});
        } catch (const std::runtime_error &e) {
            rejected = std::string(e.what()).find("not built") != std::string::npos;
        }
        assert(rejected);
        rejected = false;
        try {
            AsyncH264Encoder encoder({384, 64});
        } catch (const std::runtime_error &e) {
            rejected = std::string(e.what()).find("not built") != std::string::npos;
        }
        assert(rejected);
    }
}
