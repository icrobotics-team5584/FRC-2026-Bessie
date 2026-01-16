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
    // _turretMotorConfig.closedLoop.PositionWrappingEnabled(true);
    // _turretMotorConfig.closedLoop.PositionWrappingInputRange(-0.5,0.5);
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
        units::degree_t _crtPosition = GetTurretAngleCRT();
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

    Logger::Log("Turret/CRT Positiion", GetTurretAngleCRT());
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

units::degree_t SubTurret::GetTurretAngleCRT() {
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

units::degree_t SubTurret::GetTurretAngle() {
    return _turretMotor.GetPosition();
}

frc2::CommandPtr SubTurret::SetMotorTargetAngle(units::degree_t angle) {
    return RunOnce([this, angle] {
        if(angle > POS_LIMIT) {_turretMotor.SetPositionTarget(POS_LIMIT);}
        if(angle < NEG_LIMIT) {_turretMotor.SetPositionTarget(NEG_LIMIT);}
        if(angle < POS_LIMIT && angle > NEG_LIMIT) {_turretMotor.SetPositionTarget(angle);}
    });
}

frc2::CommandPtr SubTurret::SetTurretTargetAngle(units::degree_t angle) {
    return SetMotorTargetAngle(CalcOptimisedTurretAngle(angle));
}

units::degree_t SubTurret::CalcOptimisedTurretAngle(units::degree_t angle) {
    units::degree_t currentAngle = SubTurret::GetInstance().GetTurretAngle();

    //limit target angle to limits
    if(angle > POS_LIMIT) { angle -= 360_deg;}
    if(angle < NEG_LIMIT) { angle += 360_deg;}

    units::degree_t closestOffset = angle - currentAngle;

    // limit to +- 180 deg
    if(closestOffset > 180_deg) {
        closestOffset -= 360_deg;
    }
    if(closestOffset < 180_deg) {
        closestOffset += 360_deg;
    }

    units::degree_t finalOffset = currentAngle + closestOffset;

    if( units::math::fmod(currentAngle + closestOffset, 360.0_deg) ==
      units::math::fmod(currentAngle - closestOffset, 360.0_deg)) {
        if(finalOffset > 0_deg) {finalOffset = currentAngle - units::math::abs(closestOffset);}
        else{finalOffset = currentAngle + units::math::abs(closestOffset);}
      }

    if(finalOffset > POS_LIMIT) {
        finalOffset -= 360_deg;
    }

    if(finalOffset < NEG_LIMIT) {
        finalOffset += 360_deg;
    }

    return finalOffset;
}

void SubTurret::SetTurretTarget(units::degree_t angle) {
    _turretMotor.SetPositionTarget(angle);
}

void SubTurret::ZeroTurret() {
    SetTurretTarget(GetTurretAngleCRT());
    _turretMotor.SetPositionTarget(GetTurretAngleCRT());
}

frc2::CommandPtr SubTurret::ZeroTurretCmd() {
    return RunOnce( [this] {
    SetTurretTarget(GetTurretAngleCRT());
    _turretMotor.SetPositionTarget(GetTurretAngleCRT());
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