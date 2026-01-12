// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubShooter.h"
#include "frc/smartdashboard/SmartDashboard.h"

SubShooter::SubShooter() {
    frc::SmartDashboard::PutData("Shooter/Motor1", &_shooterMotor1);
    frc::SmartDashboard::PutData("Shooter/Motor2", &_shooterMotor2);

    _shooterMotor1Config.encoder.PositionConversionFactor(1/GEAR_RATIO);
    _shooterMotor1Config.encoder.VelocityConversionFactor(1/GEAR_RATIO/60);
    _shooterMotor1Config.closedLoop.Pid(P,I,D);
    _shooterMotor1Config.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kCoast);
    auto err1 = _shooterMotor1.AdjustConfig(_shooterMotor1Config);
    frc::SmartDashboard::PutNumber("Shooter/config set err1", (int)err1);

    _shooterMotor2Config.Follow(canid::SHOOTER_MOTOR_1, true);
    auto err2 = _shooterMotor1.AdjustConfig(_shooterMotor1Config);
    frc::SmartDashboard::PutNumber("Shooter/config set err2", (int)err2);
}

// This method will be called once per scheduler run
void SubShooter::Periodic() {
}

void SubShooter::SimulationPeriodic() {
    units::volt_t volts = _shooterMotor1.CalcSimVoltage();

    _flywheelSim.SetInputVoltage(volts);
    _flywheelSim.Update(20_ms);
    auto velocity = _flywheelSim.GetAngularVelocity();
    _shooterMotor1.IterateSim(velocity);
}

frc2::CommandPtr SubShooter::SetShooterSpeed(units::turns_per_second_t speed) {
    return RunOnce([this, speed] {_shooterMotor1.SetVelocityTarget(speed);});
}


frc2::CommandPtr SubShooter::SpinUpShooter() {
    return RunOnce([this] {_shooterMotor1.SetVelocityTarget(SHOOTER_SPEED);});
}


frc2::CommandPtr SubShooter::StopShooter() {
    return RunOnce([this] {_shooterMotor1.SetVelocityTarget(0_tps);});
}

bool SubShooter::IsAtSpeed() {
    return abs(_shooterMotor1.GetVelocity().value() - _shooterMotor1.GetVelocityTarget().value()) < 0.1 &&
    abs(_shooterMotor2.GetVelocity().value() - _shooterMotor2.GetVelocityTarget().value()) < 0.1;
}