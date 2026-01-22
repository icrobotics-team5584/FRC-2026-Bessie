// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubFeeder.h"

#include <units/current.h>
#include <utilities/Logger.h>

SubFeeder::SubFeeder() {
  _feederMotorConfig.SmartCurrentLimit(60);
  _feederMotor.OverwriteConfig(_feederMotorConfig);
  Logger::Log("Feeder/Feeder Motor", &_feederMotor);
}

frc2::CommandPtr SubFeeder::FeederOn() {
  return StartEnd([this] { _feederMotor.Set(1); }, [this] { _feederMotor.Set(0); });
}

frc2::CommandPtr SubFeeder::FeederOff() {
  return RunOnce([this] { _feederMotor.Set(0); });
}

bool SubFeeder::FeederIsFull() {
  return _feederFullSensor.Get();
}

bool SubFeeder::FeederIsEmpty() {
  return _feederEmptySensor.Get();
}

void SubFeeder::CurrentHighTimer() {
  _feederHighCurrentTimer.Start();

  if (_feederHighCurrentTimer.Get() > 3_s) {
    _feederCurrentAlert.Set(true);
  }
}

// This method will be called once per scheduler run
void SubFeeder::Periodic() {
  Logger::Log("Feeder/Feeder Is Full", FeederIsFull());
  Logger::Log("Feeder/Feeder Is Empty", FeederIsEmpty());
  units::ampere_t current = _feederMotor.GetStatorCurrent();
  Logger::Log("Feeder/Feeder Motor Current", current);
  if (current > 20_A) {
    SubFeeder::CurrentHighTimer();
  } else {
    _feederCurrentAlert.Set(false);
    _feederHighCurrentTimer.Reset();
  }

  units::celsius_t temperature = _feederMotor.GetTemperature();
  Logger::Log("Feeder/Feeder Motor Temperature", temperature);
  if (temperature > 60_degC) {
    _feederHighTemperatureAlert.Set(true);
  } else {
    _feederHighTemperatureAlert.Set(false);
  }
}

void SubFeeder::SimulationPeriodic() {
  _sim.SetInputVoltage(_feederMotor.CalcSimVoltage());
  _sim.Update(20_ms);
  _feederMotor.IterateSim(_sim.GetAngularVelocity());
}
