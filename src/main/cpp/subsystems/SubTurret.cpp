// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubTurret.h"
#include <frc/smartdashboard/SmartDashboard.h>

SubTurret::SubTurret() {
    frc::SmartDashboard::PutData("Turret/Motor", &_turretMotor);

    _turretMotorConfig.encoder.PositionConversionFactor(1/GEAR_RATIO);
    _turretMotorConfig.encoder.VelocityConversionFactor(1/GEAR_RATIO/60);
    _turretMotorConfig.closedLoop.Pid(P,I,D);
    _turretMotorConfig.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake);
    auto err1 = _turretMotor.AdjustConfig(_turretMotorConfig);
    frc::SmartDashboard::PutNumber("Turret/config set err", (int)err1);
}

// This method will be called once per scheduler run
void SubTurret::Periodic() {
    
}

void SubTurret::SimulationPeriodic() {
    
}

double SubTurret::GetTurretAngle(double encoder1, double encoder2) {
    double difference = encoder1 - encoder2;
    double angle = difference * (ENCODER1_RATIO - ENCODER2_RATIO);
    return angle;
}

frc2::CommandPtr SubTurret::SetTurretAngle(units::degree_t angle) {
    return Run([this, angle] {
        _turretMotor.SetPositionTarget(angle);
    });
}