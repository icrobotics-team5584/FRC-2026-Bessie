// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubShooter.h"
#include "frc/smartdashboard/SmartDashboard.h"
#include <ctre/phoenix6/configs/Configuration.hpp>
#include <ctre/phoenix6/controls/Follower.hpp>
#include "utilities/Logger.h"

SubShooter::SubShooter() {
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
    _shooterMotor2.SetControl(ctre::phoenix6::controls::Follower(_shooterMotor1.GetDeviceID(), ctre::phoenix6::signals::MotorAlignmentValue::Opposed));

    _shooterMotor1.GetClosedLoopReference().SetUpdateFrequency(100_Hz);
}

// This method will be called once per scheduler run
void SubShooter::Periodic() {
    Logger::LogFalcon("Shooter/Motor1", _shooterMotor1);
    Logger::LogFalcon("Shooter/Motor2", _shooterMotor2);
    frc::SmartDashboard::PutData("Shooter/mech2dDisplay", &_shooterMech);


    units::angle::degree_t motor1Position = _shooterMotor1.GetPosition().GetValue();
    _shooterMechTopRoller.SetAngle(motor1Position);

    units::angle::degree_t motor2Position = _shooterMotor2.GetPosition().GetValue();
    _shooterMechBottomRoller.SetAngle(motor2Position);
}

void SubShooter::SimulationPeriodic() {
    auto& leftState = _shooterMotor1.GetSimState();
    leftState.SetSupplyVoltage(12.0_V);

    _leftFlywheelSim.SetInputVoltage(leftState.GetMotorVoltage());
    _leftFlywheelSim.Update(20_ms);

    leftState.SetRotorVelocity(_leftFlywheelSim.GetAngularVelocity());
    leftState.SetRotorAcceleration(_leftFlywheelSim.GetAngularAcceleration());
    leftState.AddRotorPosition(_leftFlywheelSim.GetAngularVelocity().value()/(3.14*2)*360*0.02*1_tr);

    auto& rightState = _shooterMotor2.GetSimState();
    rightState.SetSupplyVoltage(12.0_V);

    _rightFlywheelSim.SetInputVoltage(rightState.GetMotorVoltage());
    _rightFlywheelSim.Update(20_ms);

    rightState.SetRotorVelocity(_rightFlywheelSim.GetAngularVelocity());
    rightState.SetRotorAcceleration(_rightFlywheelSim.GetAngularAcceleration());
    rightState.AddRotorPosition(_rightFlywheelSim.GetAngularVelocity().value()/(3.14*2)*360*0.02*1_tr);
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

frc2::CommandPtr SubShooter::SetTargetFromProjectileVel(units::meters_per_second_t speed) {
    return SetShooterTarget(_ball_to_shooter_turn[speed]);
}