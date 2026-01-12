// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubHood.h"
#include "frc/smartdashboard/SmartDashboard.h"
#include "frc/RobotBase.h"

SubHood::SubHood() {
    frc::SmartDashboard::PutData("Hood/Motor", &_hoodMotor);

    _hoodMotorConfig.encoder.PositionConversionFactor(1/GEAR_RATIO);
    _hoodMotorConfig.encoder.VelocityConversionFactor(1/GEAR_RATIO/60);
    _hoodMotorConfig.closedLoop.Pid(P, I, D);

    _pitchTable.insert(-12_deg, 13.75_deg); // molly pitchtable as dummy values
    _pitchTable.insert(-11_deg, 14.5_deg);
    _pitchTable.insert(-10_deg, 15.5_deg);
    _pitchTable.insert(-9_deg, 17_deg);
    _pitchTable.insert(-4.5_deg, 22.5_deg);
    _pitchTable.insert(0_deg, 27_deg);
    _pitchTable.insert(9_deg, 33_deg);
    _pitchTable.insert(10_deg, 34.5_deg);
}

// This method will be called once per scheduler run
void SubHood::Periodic() {

}

void SubHood::SimulationPeriodic() {
    
}

frc2::CommandPtr SubHood::SetHoodPosition(units::degree_t angle) {
    return RunOnce([this, angle] {_hoodMotor.SetPositionTarget(angle);});
}

frc2::CommandPtr SubHood::PivotFromVision(std::function<units::degree_t()> tagAngle) {
    return Run([this, tagAngle]{
        _hoodMotor.SetPositionTarget(_pitchTable[tagAngle()]);
        frc::SmartDashboard::PutNumber("Pivot/TagAngle", tagAngle().value());
    });
}


frc2::CommandPtr SubHood::ZeroHood() {
    return RunOnce([this] {_resetting = true;}).AndThen(ManualHoodDown())
    .AndThen(HoodResetCheck()).AndThen([this] {_hoodMotor.SetPosition(0_deg);})
    .FinallyDo([this] {
        _hoodMotor.StopMotor();
        _resetting = false;
    });
}

frc2::CommandPtr SubHood::HoodResetCheck() {
    return RunOnce([this] {_hasreset = false;}).AndThen(Run([this] {
        if(GetHoodMotorCurrent() > zeroingCurrentLimit) {
            _hasreset = true;
        }

        if(frc::RobotBase::IsSimulation() == true) {
            _hasreset = true;
        }
    }).Until([this] {return _hasreset;}));
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
    _hoodMotor.SetMaxMotionTarget(targRot);});
}
