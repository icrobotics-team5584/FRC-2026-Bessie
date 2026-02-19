#pragma once

#include "MotorIO.h"

class GeneralMotor {
    public:
        GeneralMotor(int motorCanID);
        void ConfigMotor();
        void SetVoltage(units::volt_t voltage);
        void SetPosition(units::degree_t pos);
        void SetPositionTarget(units::degree_t target);
        void SetBrakeMode(bool isBreakModeOn);
        void Log(std::string keyName);
        void StopMotor();
        void IterateSim(units::revolutions_per_minute_t velocity, units::turn_t position);
        units::degree_t GetPosition();
        units::degree_t GetPositionTarget();
        units::degree_t GetPositionError();
        units::ampere_t GetCurrent();
        units::volt_t GetVoltage();
        units::volt_t CalcSimVoltage();
    
    private:
        std::unique_ptr<MotorIO> _io;
};