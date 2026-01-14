// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubShooter.h"
#include "frc/smartdashboard/SmartDashboard.h"
#include <ctre/phoenix6/configs/Configuration.hpp>
#include <ctre/phoenix6/controls/Follower.hpp>

SubShooter::SubShooter() {
    frc::SmartDashboard::PutData("Shooter/Motor1", &_shooterMotor1);
    frc::SmartDashboard::PutData("Shooter/Motor2", &_shooterMotor2);

    // Coast Mode
    _shooterMotor1Config.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Coast;

    // invert motors if needed
    _shooterMotor1Config.MotorOutput.Inverted = true;

    // Current Limits no idea what to actually put here
    _shooterMotor1Config.CurrentLimits.SupplyCurrentLimitEnable = true;
    _shooterMotor1Config.CurrentLimits.SupplyCurrentLowerLimit = 20.0_A;
    _shooterMotor1Config.CurrentLimits.SupplyCurrentLimit = 60.0_A;
    _shooterMotor1Config.CurrentLimits.SupplyCurrentLowerTime = 0.5_s;
    _shooterMotor1Config.CurrentLimits.StatorCurrentLimitEnable = true;
    _shooterMotor1Config.CurrentLimits.StatorCurrentLimit = 80.0_A;

    // PIDs
    _shooterMotor1Config.Slot0.kP = P;
    _shooterMotor1Config.Slot0.kI = I;
    _shooterMotor1Config.Slot0.kD = D;
    _shooterMotor1Config.Slot0.kV = V;

    // Feedback Sensor Ratio
    _shooterMotor1Config.Feedback.SensorToMechanismRatio = GEAR_RATIO;

    _shooterMotor1.GetConfigurator().Apply(_shooterMotor1Config);
    _shooterMotor2.GetConfigurator().Apply(_shooterMotor1Config);

    // Set motor 2 to follow motor 1
    _shooterMotor2.SetControl(ctre::phoenix6::controls::Follower(_shooterMotor1.GetDeviceID(), false));

    _shooterMotor1.GetClosedLoopReference().SetUpdateFrequency(100_Hz);
}

// This method will be called once per scheduler run
void SubShooter::Periodic() {
}

void SubShooter::SimulationPeriodic() {
    // units::volt_t volts = _shooterMotor1.CalcSimVoltage();
    auto& leftState = _shooterMotor1.GetSimState();
    _flywheelSim.SetInputVoltage(leftState.GetMotorVoltage());
    _flywheelSim.Update(20_ms);
    units::volt_t volts = _shooterMotor1.GetMotorVoltage().GetValue();
    leftState.SetSupplyVoltage(volts);
    leftState.SetRotorVelocity(_flywheelSim.GetAngularVelocity());
    leftState.SetRotorAcceleration(_flywheelSim.GetAngularAcceleration());

    // auto velocity = _flywheelSim.GetAngularVelocity();
    // _shooterMotor1.IterateSim(velocity);
}

frc2::CommandPtr SubShooter::SetShooterTarget(units::turns_per_second_t speed) {
    return RunOnce([this, speed] {_shooterMotor1.SetControl(_flywheelTargetVelocity.WithVelocity(speed));});
}

frc2::CommandPtr SubShooter::StopShooter() {
    return RunOnce([this] {_shooterMotor1.SetControl(_flywheelTargetVelocity.WithVelocity(0_tps));});
}

bool SubShooter::IsAtSpeed() {
    return abs(_shooterMotor1.GetVelocity().GetValueAsDouble() - _flywheelTargetVelocity.Velocity()) < 1.0 &&
    abs(_shooterMotor2.GetVelocity().GetValueAsDouble() - _flywheelTargetVelocity.Velocity()) < 1.0;
}