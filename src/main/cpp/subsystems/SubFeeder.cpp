// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubFeeder.h"

#include <frc/Alert.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <units/current.h>
#include <utilities/Logger.h>

SubFeeder::SubFeeder() {
  _feederMotorConfig.SmartCurrentLimit(60);
  _feederMotor.OverwriteConfig(_feederMotorConfig);
  Logger::Log("Feeder/Feeder Motor", &_feederMotor);
}

frc2::CommandPtr SubFeeder::FeederOn() {
  return StartEnd([this] { _feederMotor.Set(1); }, [this] { _feederMotor.Set(0); });
};

frc2::CommandPtr SubFeeder::FeederOff() {
  return RunOnce([this] { _feederMotor.Set(0); });
};

void SubFeeder::CurrentHighTimer() {
  _feederHighCurrentTimer.Start();

  if (_feederHighCurrentTimer.Get() > 3_s) {
    feederCurrentAlert.Set(true);
  }
};

// This method will be called once per scheduler run
void SubFeeder::Periodic() {
  units::ampere_t current = _feederMotor.GetOutputCurrent() * 1_A;
  frc::SmartDashboard::PutNumber("Feeder/Current", current.value());
  if (current > 20_A) {
    SubFeeder::CurrentHighTimer();
  } else {
    feederCurrentAlert.Set(false);
    _feederHighCurrentTimer.Reset();
  }

  units::celsius_t temperature = _feederMotor.GetTemperature();
  frc::SmartDashboard::PutNumber("Feeder/FeederMotor Temperature", temperature.value());
  if (temperature > 60_degC) {
    highTempuratureAlert.Set(true);
  } else {
    highTempuratureAlert.Set(false);
  }
}

void SubFeeder::SimulationPeriodic() {
  _sim.SetInputVoltage(_feederMotor.CalcSimVoltage());
  _sim.Update(20_ms);
  _feederMotor.IterateSim(_sim.GetAngularVelocity());
}
