// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubHood.h"
#include "frc/smartdashboard/SmartDashboard.h"
#include "frc/RobotBase.h"
#include "utilities/Logger.h"

SubHood::SubHood() {

    _hoodMotorConfig.encoder.PositionConversionFactor(1/GEAR_RATIO);
    _hoodMotorConfig.encoder.VelocityConversionFactor(1/GEAR_RATIO);
    _hoodMotorConfig.closedLoop.Pid(P, I, D);
    _hoodMotorConfig.SmartCurrentLimit(30);
    _hoodMotor.OverwriteConfig(_hoodMotorConfig);

    frc::SmartDashboard::PutData("Hood/Motor", &_hoodMotor);
    frc::SmartDashboard::PutData("Hood/mech2dDisplay", &_hoodMech);
}

// This method will be called once per scheduler run
void SubHood::Periodic() {
    _hoodMechCircle.SetAngle(_hoodMotor.GetPosition());
    if (_hasZeroed == false && _zeroing == false) {
        _hoodMotor.Set(0);
    }

    Logger::Log("Hood/haszeroed", _hasZeroed);
    Logger::Log("Hood/zeroing", _zeroing);
}

void SubHood::SimulationPeriodic() {
    _hoodSim.SetInputVoltage(_hoodMotor.CalcSimVoltage());
    _hoodSim.Update(20_ms);
    _hoodMotor.IterateSim(_hoodSim.GetVelocity(), _hoodSim.GetAngle());
}

frc2::CommandPtr SubHood::SetHoodPositionTarget(units::degree_t angle) {
    return RunOnce([this, angle] {_hoodMotor.SetPositionTarget(angle);});
}

frc2::CommandPtr SubHood::ZeroHood() {
    return RunOnce([this] {_zeroing = true;}).AndThen(ManualHoodDown())
    .Until([this] {return HoodCurrentCheck();})
    .AndThen([this] {_hoodMotor.SetPosition(LOWER_LIMIT);})
    .FinallyDo([this] {
        _hoodMotor.StopMotor();
        _hoodMotor.SetPositionTarget(LOWER_LIMIT);
        _zeroing = false;
    });
}

bool SubHood::HoodCurrentCheck() {
    _hasZeroed = false;
    if(units::math::abs(GetHoodMotorCurrent()) > zeroingCurrentLimit) {
        _hasZeroed = true;
        return true;
    }

    return false;
}

units::ampere_t SubHood::GetHoodMotorCurrent() {
    return _hoodMotor.GetOutputCurrent()*1_A;
}

frc2::CommandPtr SubHood::StowHood() {
    return RunOnce([this] {_hoodMotor.SetPositionTarget(STOW_ANGLE);});
}

frc2::CommandPtr SubHood::ManualHoodDown() {
    return StartEnd([this] {_hoodMotor.SetVoltage(-1_V);}, 
    [this] {auto targRot = _hoodMotor.GetPosition();
    _hoodMotor.SetPositionTarget(targRot);});
}

units::degree_t SubHood::GetHoodAngle(){
    return _hoodMotor.GetPosition();
}
