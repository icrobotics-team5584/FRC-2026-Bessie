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
    _shooterMotorConfig.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Coast;

    // invert motors if needed
    _shooterMotorConfig.MotorOutput.Inverted = true;

    // Current Limits no idea what to actually put here
    _shooterMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    _shooterMotorConfig.CurrentLimits.SupplyCurrentLowerLimit = 20.0_A;
    _shooterMotorConfig.CurrentLimits.SupplyCurrentLimit = 60.0_A;
    _shooterMotorConfig.CurrentLimits.SupplyCurrentLowerTime = 0.5_s;
    _shooterMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    _shooterMotorConfig.CurrentLimits.StatorCurrentLimit = 80.0_A;

    // PIDs
    _shooterMotorConfig.Slot0.kP = P;
    _shooterMotorConfig.Slot0.kI = I;
    _shooterMotorConfig.Slot0.kD = D;
    _shooterMotorConfig.Slot0.kV = V;

    // Feedback Sensor Ratio
    _shooterMotorConfig.Feedback.SensorToMechanismRatio = GEAR_RATIO;
    // Set motor 2 to follow motor 1
    _shooterMotor2.SetControl(ctre::phoenix6::controls::Follower(_shooterMotor1.GetDeviceID(), ctre::phoenix6::signals::MotorAlignmentValue::Opposed));

    _shooterMotor1.GetConfigurator().Apply(_shooterMotorConfig);
    _shooterMotor2.GetConfigurator().Apply(_shooterMotorConfig);

    _shooterMotor1.GetClosedLoopReference().SetUpdateFrequency(100_Hz);

    _timeOfFlightTable.insert(1270_mm, 0.8_s);
    _timeOfFlightTable.insert(1770_mm, 0.5_s);
    _timeOfFlightTable.insert(2270_mm, 0.93_s);
    _timeOfFlightTable.insert(2770_mm, 1.1_s);
    _timeOfFlightTable.insert(3770_mm, 1.18_s);
    _timeOfFlightTable.insert(3770_mm, 1.18_s);
    _timeOfFlightTable.insert(4270_mm, 1.28_s);
    _timeOfFlightTable.insert(4770_mm, 1.28_s);

    frc::SmartDashboard::PutData("Shooter/mech2dDisplay", &_shooterMech);

    _flyWheelSpeedTable.insert(1.8575_m, 26_tps);
    _flyWheelSpeedTable.insert(2.3575_m, 27_tps);
    _flyWheelSpeedTable.insert(2.8575_m, 29_tps);
    _flyWheelSpeedTable.insert(3.3575_m, 33_tps);
    _flyWheelSpeedTable.insert(3.8575_m, 35_tps);
    _flyWheelSpeedTable.insert(4.3575_m, 38.5_tps);
    _flyWheelSpeedTable.insert(4.6875_m, 41_tps);
    _flyWheelSpeedTable.insert(5.1875_m, 44_tps);
}

// This method will be called once per scheduler run
void SubShooter::Periodic() {
    Logger::LogFalcon("Shooter/Motor1", _shooterMotor1);
    Logger::LogFalcon("Shooter/Motor2", _shooterMotor2);
    Logger::Log("Shooter/IsAtSpeed", IsAtSpeed());

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
    leftState.AddRotorPosition(_leftFlywheelSim.GetAngularVelocity()*20_ms);

    auto& rightState = _shooterMotor2.GetSimState();
    rightState.SetSupplyVoltage(12.0_V);

    _rightFlywheelSim.SetInputVoltage(rightState.GetMotorVoltage());
    _rightFlywheelSim.Update(20_ms);

    rightState.SetRotorVelocity(_rightFlywheelSim.GetAngularVelocity());
    rightState.SetRotorAcceleration(_rightFlywheelSim.GetAngularAcceleration());
    rightState.AddRotorPosition(_rightFlywheelSim.GetAngularVelocity()*20_ms);
}

frc2::CommandPtr SubShooter::SetShooterTarget(std::function<units::turns_per_second_t()> speed) {
    return Run([this, speed] {_shooterMotor1.SetControl(_flywheelTargetVelocity.WithVelocity(speed()));});
}

frc2::CommandPtr SubShooter::StopShooter() {
    return RunOnce([this] {_shooterMotor1.SetControl(_flywheelTargetVelocity.WithVelocity(0_tps));});
}

bool SubShooter::IsAtSpeed() {
    return units::math::abs(_shooterMotor1.GetVelocity().GetValue() - _flywheelTargetVelocity.Velocity) < 1.0_tps &&
    units::math::abs(_shooterMotor2.GetVelocity().GetValue() - _flywheelTargetVelocity.Velocity) < 1.0_tps;
}

units::revolutions_per_minute_t SubShooter::GetShooterSpeed(){
    auto vel = _shooterMotor1.GetVelocity().GetValue();
    return vel*60;
}

frc2::CommandPtr SubShooter::SpinWithDistance(std::function<units::meter_t()> distance) {
    return SetShooterTarget([this, distance] { return _flyWheelSpeedTable[distance()]; });
}

units::second_t SubShooter::GetTimeOfFLightWithDistance(units::meter_t distance) {
    return _timeOfFlightTable[distance];
}
