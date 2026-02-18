#pragma once

#include <ctre/phoenix6/CANcoder.hpp>
#include <frc/DutyCycleEncoder.h>

class EncoderIO {
    public:
        virtual void ConfigEncoder() = 0;
        virtual units::degree_t GetPosition() = 0;
};