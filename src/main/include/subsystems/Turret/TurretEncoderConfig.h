#pragma once

#include <units/frequency.h>

namespace TurretThroughboreConfig {
    static constexpr units::hertz_t ENCODER_FREQUENCY = 975.6_Hz;
    const double encoder1ZeroOffset = 0.696408;
    const double encoder2ZeroOffset = 0.120609;
}

namespace TurretCancoderConfig {
    const double encoder1ZeroOffset = 0.696408;
    const double encoder2ZeroOffset = 0.120609;
}