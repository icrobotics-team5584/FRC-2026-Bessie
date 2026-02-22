#pragma once

#include <ctre/phoenix6/CANcoder.hpp>
#include <frc/DutyCycleEncoder.h>

class TurretEncoderIO {
    public:
        virtual void ConfigEncoder() = 0;
        virtual double GetPosition() = 0;
        virtual bool IsConnected() = 0;
};