// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubTurret.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include "utilities/Logger.h"

SubTurret::SubTurret() {
    // Logger::Log("Turret/CRT Positiion", GetTurretAngle());

    _turretMotorConfig.encoder.PositionConversionFactor(1/GEAR_RATIO);
    _turretMotorConfig.encoder.VelocityConversionFactor(1/GEAR_RATIO/60);
    _turretMotorConfig.closedLoop.Pidf(P, I, D, F);
    _turretMotorConfig.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake);
    _turretMotorConfig.SmartCurrentLimit(30);
    _turretMotor.OverwriteConfig(_turretMotorConfig);
}

// This method will be called once per scheduler run
void SubTurret::Periodic() {
    frc::SmartDashboard::PutData("Turret/Motor", &_turretMotor);
}

void SubTurret::SimulationPeriodic() {
    frc::SmartDashboard::PutData("Turret/mech2dDisplay", &_turretMech);

    _turretSim.SetInputVoltage(_turretMotor.CalcSimVoltage());
    _turretMotor.IterateSim(_turretSim.GetAngularVelocity(), _turretSim.GetAngularPosition());
    _turretSim.Update(20_ms);

    _turretMechCircle.SetAngle(_turretMotor.GetPosition());
}

// double SubTurret::GetTurretAngle() {
//     double encoder1 = _turretEncoder1.GetDistance();
//     double encoder2 = _turretEncoder2.GetDistance();

//     double difference = encoder1 - encoder2;
//     double angle = difference * (ENCODER1_RATIO - ENCODER2_RATIO);
//     return angle;
// }

frc2::CommandPtr SubTurret::SetTurretAngle(units::degree_t angle) {
    return Run([this, angle] {
        _turretMotor.SetPositionTarget(angle);
    });
}