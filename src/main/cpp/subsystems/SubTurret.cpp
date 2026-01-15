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
    _turretMotorConfig.closedLoop.feedForward.kV(F);
    _turretMotorConfig.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kCoast);
    _turretMotorConfig.SmartCurrentLimit(30);
    _turretMotorConfig.closedLoop.PositionWrappingEnabled(true);
    _turretMotorConfig.closedLoop.PositionWrappingInputRange(-0.5,0.5);
    _turretMotor.OverwriteConfig(_turretMotorConfig);
}

// This method will be called once per scheduler run
void SubTurret::Periodic() {
    frc::SmartDashboard::PutData("Turret/Motor", &_turretMotor);
    Logger::Log("Turret/CRT Positiion", GetTurretAngle());
    Logger::Log("Turret/Encoder1", _turretEncoder1.Get());
    Logger::Log("Turret/Encoder2", _turretEncoder2.Get());
}

void SubTurret::SimulationPeriodic() {
    frc::SmartDashboard::PutData("Turret/mech2dDisplay", &_turretMech);

    _turretSim.SetInputVoltage(_turretMotor.CalcSimVoltage());
    _turretMotor.IterateSim(_turretSim.GetAngularVelocity(), _turretSim.GetAngularPosition());
    _turretSim.Update(20_ms);

    _turretMechCircle.SetAngle(_turretMotor.GetPosition());
}

// units::turn_t SubTurret::GetTurretAngle() {
//     double encoder1 = _turretEncoder1.Get();
//     double encoder2 = _turretEncoder2.Get();

//     double difference = encoder1 - encoder2;
//     double angle = difference * (ENCODER1_RATIO - ENCODER2_RATIO);
//     return angle*1_tr;
// }

units::degree_t SubTurret::GetTurretAngle() {
    double e1deg = _turretEncoder1.Get() * 360.0;
    double e2deg = _turretEncoder2.Get() * 360.0;
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

void SubTurret::SetAngle(units::degree_t angle) {
    _turretMotor.SetPosition(angle);
}

frc2::CommandPtr SubTurret::SetTurretTarget(units::degree_t angle) {
    return Run([this, angle] {
        SetTarget(angle);
    });
}

void SubTurret::SetTarget(units::degree_t angle) {
    _turretMotor.SetPositionTarget(angle);
}

frc2::CommandPtr SubTurret::SetTurretAngle(units::degree_t angle) {
    return RunOnce([this, angle] {
        SetAngle(angle);
    });
}

frc2::CommandPtr SubTurret::ZeroTurret() {
    return RunOnce([this] {SetTarget(GetTurretAngle());});
}