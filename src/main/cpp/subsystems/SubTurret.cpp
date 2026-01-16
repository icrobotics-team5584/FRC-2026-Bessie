// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubTurret.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include "utilities/Logger.h"

SubTurret::SubTurret() {
    _turretMotorConfig.encoder.PositionConversionFactor(1/GEAR_RATIO);
    _turretMotorConfig.encoder.VelocityConversionFactor(1/GEAR_RATIO);
    _turretMotorConfig.closedLoop.Pid(P, I, D);
    _turretMotorConfig.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kCoast);
    _turretMotorConfig.SmartCurrentLimit(30);
    _turretMotor.OverwriteConfig(_turretMotorConfig);

    _turretEncoder1.SetAssumedFrequency(975.6_Hz);
    _turretEncoder2.SetAssumedFrequency(975.6_Hz);

    frc::SmartDashboard::PutData("Turret/Motor", &_turretMotor);
    frc::SmartDashboard::PutData("Turret/mech2dDisplay", &_turretMech);
}

// This method will be called once per scheduler run
void SubTurret::Periodic() {

    if(_hasReset == false && _turretEncoder1.IsConnected() && _turretEncoder2.IsConnected()) {
        units::degree_t _motorPosition = _turretMotor.GetPosition();
        units::degree_t _crtPosition = GetTurretAngle();
        Logger::Log("Turret/reset/_motorPosition", _motorPosition);
        Logger::Log("Turret/reset/_crtPosition", _crtPosition);

        bool CRTSameAsMotor = (units::math::abs(_motorPosition - _crtPosition) < 0.5_deg);
        Logger::Log("Turret/CRTSameAsMotor", CRTSameAsMotor);

        if(CRTSameAsMotor){
            _hasReset = true;
        }
        if(!CRTSameAsMotor){
            _hasReset = false;
            ZeroTurret();
        }
    }

    _turretMechCircle.SetAngle(_turretMotor.GetPosition());

    Logger::Log("Turret/CRT Positiion", GetTurretAngle());
    Logger::Log("Turret/Encoder/Encoder1", _turretEncoder1.Get());
    Logger::Log("Turret/Encoder/Encoder2", _turretEncoder2.Get());
    Logger::Log("Turret/Encoder/ZeroedEncoder1", getEncoder1Degrees());
    Logger::Log("Turret/Encoder/ZeroedEncoder2", getEncoder2Degrees());
    Logger::Log("Turret/Encoder/e1init", E1initial);
    Logger::Log("Turret/Encoder/e2init", E2initial);
    Logger::Log("Turret/hasReset", _hasReset);

    Logger::Log("Turret/Encoder/Encoder1IsConnected", _turretEncoder1.IsConnected());
    Logger::Log("Turret/Encoder/Encoder2IsConnected", _turretEncoder2.IsConnected());
    Logger::Log("Turret/Encoder/Encoder1Frequency", _turretEncoder1.GetFrequency());
    Logger::Log("Turret/Encoder/Encoder2Frequency", _turretEncoder2.GetFrequency());
}

void SubTurret::SimulationPeriodic() {
    _turretSim.SetInputVoltage(_turretMotor.CalcSimVoltage());
    _turretSim.Update(20_ms);
    _turretMotor.IterateSim(_turretSim.GetAngularVelocity(), _turretSim.GetAngularPosition());
}

units::degree_t SubTurret::GetTurretAngle() {
    double e1deg = getEncoder1Degrees().value();
    double e2deg = getEncoder2Degrees().value();
    double difference = e2deg - e1deg;

    if(difference > 180) {
        difference -= 360;
    } else if(difference < -180) {
        difference += 360;
    } 

    double SLOPE = (E2_TEETH * E1_TEETH) / (BIG_TOOTH);
    difference *= SLOPE;

    double e1rotations = (difference * BIG_TOOTH / E1_TEETH) / 360.0;
    double e1rotations_floored = floor(e1rotations);

    double turretAngle = (
        (e1rotations_floored * 360.0 + e1deg) *
        (E1_TEETH / BIG_TOOTH)
    );

    double period = (E1_TEETH / BIG_TOOTH) * 360.0;

    if(turretAngle - difference < -period / 2) {
        turretAngle += period;
    } else if(turretAngle - difference > period / 2) {
        turretAngle -= period;
    }

    return turretAngle * 1_deg;
}

frc2::CommandPtr SubTurret::SetTurretTargetAngle(units::degree_t angle) {
    return RunOnce([this, angle] {
        _turretMotor.SetPositionTarget(angle);
    });
}

void SubTurret::SetTurretTarget(units::degree_t angle) {
    _turretMotor.SetPositionTarget(angle);
}

void SubTurret::SetTurretAngle(units::degree_t angle) {
    _turretMotor.SetPosition(angle);
}

void SubTurret::ZeroTurret() {
    SetTurretAngle(GetTurretAngle());
    _turretMotor.SetPositionTarget(GetTurretAngle());
}

frc2::CommandPtr SubTurret::ZeroTurretCmd() {
    return RunOnce( [this] {
    SetTurretAngle(GetTurretAngle());
    _turretMotor.SetPositionTarget(GetTurretAngle());
    });
}

units::degree_t SubTurret::getEncoder1Degrees() {
    return (_turretEncoder1.Get()-E1initial)*360_deg;
}

units::degree_t SubTurret::getEncoder2Degrees() {
    return (_turretEncoder2.Get()-E2initial)*360_deg;
}

frc2::CommandPtr SubTurret::zeroEncoders() {
    return RunOnce([this] {
        E1initial = _turretEncoder1.Get();
        E2initial = _turretEncoder2.Get();
    });
}