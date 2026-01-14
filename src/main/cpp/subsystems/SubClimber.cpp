// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubClimber.h"

SubClimber::SubClimber() {
    _topClimbMotorConfig.encoder.PositionConversionFactor(GEAR_RATIO);
    _topClimbMotorConfig.encoder.VelocityConversionFactor(GEAR_RATIO / 60); /* RPM / 60 */
    _topClimbMotorConfig.SmartCurrentLimit(60); /* Amps */ 
    _topClimbMotorConfig.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake);
    _topClimbMotorConfig.closedLoop.Pid(P, I, D, rev::spark::ClosedLoopSlot::kSlot0);
    _topClimbMotor.OverwriteConfig(_topClimbMotorConfig);

    _botClimbMotorConfig.encoder.PositionConversionFactor(GEAR_RATIO);
    _botClimbMotorConfig.encoder.VelocityConversionFactor(GEAR_RATIO / 60); /* RPM / 60 */
    _botClimbMotorConfig.SmartCurrentLimit(60); /* Amps */ 
    _topClimbMotorConfig.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake);
    _botClimbMotorConfig.closedLoop.Pid(P, I, D, rev::spark::ClosedLoopSlot::kSlot0);
    _botClimbMotor.OverwriteConfig(_botClimbMotorConfig);
    
    frc::SmartDashboard::PutData("Climber/topClimbMotor", &_topClimbMotor);
    frc::SmartDashboard::PutData("Climber/bottomClimbMotor", &_botClimbMotor);
}

// This method will be called once per scheduler run
void SubClimber::Periodic() {}

void SimulationPeriodic() {}
