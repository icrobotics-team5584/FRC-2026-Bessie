// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubHood.h"
#include "frc/smartdashboard/SmartDashboard.h"
#include "frc/RobotBase.h"

SubHood::SubHood() {

    _hoodMotorConfig.encoder.PositionConversionFactor(1/GEAR_RATIO);
    _hoodMotorConfig.encoder.VelocityConversionFactor(1/GEAR_RATIO);
    _hoodMotorConfig.closedLoop.Pid(P, I, D);
    _hoodMotorConfig.SmartCurrentLimit(30);
    _hoodMotor.OverwriteConfig(_hoodMotorConfig);

    _pitchTable.insert(1_m, 12.5_deg); // dummy values
    _pitchTable.insert(2_m, 35.0_deg);

    frc::SmartDashboard::PutData("Hood/Motor", &_hoodMotor);
    frc::SmartDashboard::PutData("Hood/mech2dDisplay", &_hoodMech);
}

// This method will be called once per scheduler run
void SubHood::Periodic() {
    _hoodMechCircle.SetAngle(_hoodMotor.GetPosition());
}

void SubHood::SimulationPeriodic() {
    _hoodSim.SetInputVoltage(_hoodMotor.CalcSimVoltage());
    _hoodSim.Update(20_ms);
    _hoodMotor.IterateSim(_hoodSim.GetVelocity(), _hoodSim.GetAngle());
}

frc2::CommandPtr SubHood::SetHoodPositionTarget(units::degree_t angle) {
    return RunOnce([this, angle] {_hoodMotor.SetPositionTarget(angle);});
}

frc2::CommandPtr SubHood::PivotFromVision(std::function<units::meter_t()> distance) {
    return Run([this, distance]{
        _hoodMotor.SetPositionTarget(_pitchTable[distance()]);
    });
}

frc2::CommandPtr SubHood::ZeroHood() {
    return RunOnce([this] {_resetting = true;}).AndThen(ManualHoodDown())
    .Until([this] {return HoodCurrentCheck();})
    .AndThen([this] {_hoodMotor.SetPosition(12.5_deg);})
    .FinallyDo([this] {
        _hoodMotor.StopMotor();
        _hoodMotor.SetPositionTarget(12.5_deg);
        _resetting = false;
    });
}

bool SubHood::HoodCurrentCheck() {
    _hasreset = false;
    if(units::math::abs(GetHoodMotorCurrent()) > zeroingCurrentLimit) {
        _hasreset = true;
        return true;
    }

    return false;
}

units::ampere_t SubHood::GetHoodMotorCurrent() {
    return _hoodMotor.GetOutputCurrent()*1_A;
}

frc2::CommandPtr SubHood::StowHood() {
    return RunOnce([this] {_hoodMotor.SetPositionTarget(STOW_TURNS);});
}

frc2::CommandPtr SubHood::ManualHoodDown() {
    return StartEnd([this] {_hoodMotor.SetVoltage(-1_V);}, 
    [this] {auto targRot = _hoodMotor.GetPosition();
    _hoodMotor.SetPositionTarget(targRot);});
}
