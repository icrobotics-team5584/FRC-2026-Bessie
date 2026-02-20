// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubShooter.h"

#include "utilities/Logger.h"

#include <ctre/phoenix6/configs/Configuration.hpp>
#include <ctre/phoenix6/controls/Follower.hpp>

#include "frc/smartdashboard/SmartDashboard.h"

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
  _shooterMotor2.SetControl(ctre::phoenix6::controls::Follower(
    _shooterMotor1.GetDeviceID(), ctre::phoenix6::signals::MotorAlignmentValue::Opposed));

  _shooterMotor1.GetConfigurator().Apply(_shooterMotorConfig);
  _shooterMotor2.GetConfigurator().Apply(_shooterMotorConfig);

  _shooterMotor1.GetClosedLoopReference().SetUpdateFrequency(100_Hz);

    _timeOfFlightTable.insert(1.8575_m, 0.8_s);
    _timeOfFlightTable.insert(2.3575_m, 0.5_s);
    _timeOfFlightTable.insert(2.8575_m, 0.93_s);
    _timeOfFlightTable.insert(3.3575_m, 1.1_s);
    _timeOfFlightTable.insert(3.8575_m, 1.18_s);
    _timeOfFlightTable.insert(4.3575_m, 1.18_s);
    _timeOfFlightTable.insert(4.6875_m, 1.28_s);
    _timeOfFlightTable.insert(5.1875_m, 1.28_s);

  frc::SmartDashboard::PutData("Shooter/mech2dDisplay", &_shooterMech);

  _flyWheelSpeedTableScoring.insert(1.8575_m, 26_tps);
  _flyWheelSpeedTableScoring.insert(2.3575_m, 27_tps);
  _flyWheelSpeedTableScoring.insert(2.8575_m, 29_tps);
  _flyWheelSpeedTableScoring.insert(3.3575_m, 33_tps);
  _flyWheelSpeedTableScoring.insert(3.8575_m, 35_tps);
  _flyWheelSpeedTableScoring.insert(4.3575_m, 38.5_tps);
  _flyWheelSpeedTableScoring.insert(4.6875_m, 41_tps);
  _flyWheelSpeedTableScoring.insert(5.1875_m, 44_tps);

  _flyWheelSpeedTablePassing.insert(5_m, 40_tps);
  _flyWheelSpeedTablePassing.insert(6_m, 45_tps);
  _flyWheelSpeedTablePassing.insert(7_m, 50_tps);
  _flyWheelSpeedTablePassing.insert(8_m, 55_tps);
}

// This method will be called once per scheduler run
void SubShooter::Periodic() {
  auto loopStart = frc::GetTime();

  Logger::LogFalcon("Shooter/Motor1", _shooterMotor1);
  Logger::LogFalcon("Shooter/Motor2", _shooterMotor2);
  Logger::Log("Shooter/IsAtSpeed", IsAtSpeed());

  units::angle::degree_t motor1Position = _shooterMotor1.GetPosition().GetValue();
  _shooterMechTopRoller.SetAngle(motor1Position);

  units::angle::degree_t motor2Position = _shooterMotor2.GetPosition().GetValue();
  _shooterMechBottomRoller.SetAngle(motor2Position);

  units::celsius_t shooter1Temperature = _shooterMotor1.GetDeviceTemp().GetValue();
  units::ampere_t shooter1Current = _shooterMotor1.GetStatorCurrent().GetValue();

  AlertController::UpdateTemperatureAlert(_shooter1AlertConfig, shooter1Temperature);
  AlertController::UpdateCurrentAlert(_shooter1AlertConfig, shooter1Current);

  units::celsius_t shooter2Temperature = _shooterMotor2.GetDeviceTemp().GetValue();
  units::ampere_t shooter2Current = _shooterMotor2.GetStatorCurrent().GetValue();

  AlertController::UpdateTemperatureAlert(_shooter2AlertConfig, shooter2Temperature);
  AlertController::UpdateCurrentAlert(_shooter2AlertConfig, shooter2Current);

  Logger::Log("Shooter/Loop Time", (frc::GetTime() - loopStart));
}

void SubShooter::SimulationPeriodic() {
  auto& leftState = _shooterMotor1.GetSimState();
  leftState.SetSupplyVoltage(12.0_V);

  _leftFlywheelSim.SetInputVoltage(leftState.GetMotorVoltage());
  _leftFlywheelSim.Update(20_ms);

  leftState.SetRotorVelocity(_leftFlywheelSim.GetAngularVelocity());
  leftState.SetRotorAcceleration(_leftFlywheelSim.GetAngularAcceleration());
  leftState.AddRotorPosition(_leftFlywheelSim.GetAngularVelocity() * 20_ms);

  auto& rightState = _shooterMotor2.GetSimState();
  rightState.SetSupplyVoltage(12.0_V);

  _rightFlywheelSim.SetInputVoltage(rightState.GetMotorVoltage());
  _rightFlywheelSim.Update(20_ms);

  rightState.SetRotorVelocity(_rightFlywheelSim.GetAngularVelocity());
  rightState.SetRotorAcceleration(_rightFlywheelSim.GetAngularAcceleration());
  rightState.AddRotorPosition(_rightFlywheelSim.GetAngularVelocity() * 20_ms);
}

frc2::CommandPtr SubShooter::SetShooterTarget(std::function<units::turns_per_second_t()> speed) {
  return Run(
    [this, speed] { _shooterMotor1.SetControl(_flywheelTargetVelocity.WithVelocity(speed())); });
}

frc2::CommandPtr SubShooter::StopShooter() {
  return RunOnce(
    [this] { _shooterMotor1.SetControl(_flywheelTargetVelocity.WithVelocity(0_tps)); });
}

frc2::CommandPtr SubShooter::SpinShooterSlowly() {
    return Run([this] {_shooterMotor1.SetControl(_flywheelTargetVelocity.WithVelocity(10_tps));});
}

bool SubShooter::IsAtSpeed() {
  return units::math::abs(
           _shooterMotor1.GetVelocity().GetValue() - _flywheelTargetVelocity.Velocity) < 1.0_tps &&
         units::math::abs(
           _shooterMotor2.GetVelocity().GetValue() - _flywheelTargetVelocity.Velocity) < 1.0_tps;
}

frc2::CommandPtr SubShooter::SpinWithDistance(
  std::function<units::meter_t()> distance, std::function<bool()> isPassing) {
  return SetShooterTarget([this, distance, isPassing] {
    return isPassing() ? _flyWheelSpeedTablePassing[distance()]
                       : _flyWheelSpeedTableScoring[distance()];
  });
}

units::second_t SubShooter::GetTimeOfFLightWithDistance(units::meter_t distance) {
  return _timeOfFlightTable[distance];
}
