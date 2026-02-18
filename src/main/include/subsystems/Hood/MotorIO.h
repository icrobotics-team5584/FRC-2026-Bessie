#pragma once

#include <ctre/phoenix6/TalonFX.hpp>
#include <units/angle.h>
#include <units/current.h>
#include <units/voltage.h>

#include "utilities/ICSparkMax.h"
#include "utilities/Logger.h"

class MotorIO {
    public:
        virtual void ConfigMotor() = 0;
        virtual void SetVoltage(units::volt_t voltage) = 0;
        virtual void SetPosition(units::degree_t pos) = 0;
        virtual void SetPositionTarget(units::degree_t target) = 0;
        virtual void SetBrakeMode(bool isBreakModeOn) = 0;
        virtual void StartLoggingMotor(std::string keyName) = 0;
        virtual void StopMotor() = 0;
        virtual units::degree_t GetPosition() = 0;
        virtual units::degree_t GetPositionTarget() = 0;
        virtual units::degree_t GetPositionError() = 0;
        virtual units::ampere_t GetCurrent() = 0;
        virtual units::volt_t GetVoltage() = 0;
};