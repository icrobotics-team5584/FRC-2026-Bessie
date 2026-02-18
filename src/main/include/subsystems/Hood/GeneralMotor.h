#pragma once

#include "MotorIO.h"

class GeneralMotor {
    public:
        GeneralMotor(int motorCanID);
        void ConfigMotor();
        void SetVoltage(units::volt_t voltage);
        void SetPositionTarget(units::degree_t target);
        void SetBrakeMode(bool isBreakModeOn);
        void StartLoggingMotor(std::string keyName);
        void StopMotor();
        units::degree_t GetPosition();
        units::degree_t GetPositionTarget();
        units::ampere_t GetCurrent();
        units::volt_t GetVoltage();
    
    private:
        std::unique_ptr<MotorIO> _io;
};