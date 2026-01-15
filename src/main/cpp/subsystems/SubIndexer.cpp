// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubIndexer.h"

#include <units/current.h>
#include <utilities/Logger.h>

SubIndexer::SubIndexer() {
  _IndexerMotorConfig.SmartCurrentLimit(60);
  _IndexerMotor.OverwriteConfig(_IndexerMotorConfig);
  Logger::Log("Indexer/Indexer Motor", &_IndexerMotor);
}

frc2::CommandPtr SubIndexer::IndexerOn() {
  return StartEnd([this] { _IndexerMotor.Set(1); }, [this] { _IndexerMotor.Set(0); });
};

frc2::CommandPtr SubIndexer::IndexerOff() {
  return RunOnce([this] { _IndexerMotor.Set(0); });
};

void SubIndexer::CurrentHighTimer() {
  _IndexerHighCurrentTimer.Start();

  if (_IndexerHighCurrentTimer.Get() > 3_s) {
    IndexerCurrentAlert.Set(true);
  }
};

// This method will be called once per scheduler run
void SubIndexer::Periodic() {
  units::ampere_t current = _IndexerMotor.GetOutputCurrent() * 1_A;
  Logger::Log("Indexer/Indexer Motor Current", current);
  if (current > 20_A) {
    SubIndexer::CurrentHighTimer();
  } else {
    IndexerCurrentAlert.Set(false);
    _IndexerHighCurrentTimer.Reset();
  }

  units::celsius_t temperature = _IndexerMotor.GetTemperature();
  Logger::Log("Indexer/IndexerMotor Temperature", temperature);
  if (temperature > 60_degC) {
    highTempuratureAlert.Set(true);
  } else {
    highTempuratureAlert.Set(false);
  }
}

void SubIndexer::SimulationPeriodic() {
  _sim.SetInputVoltage(_IndexerMotor.CalcSimVoltage());
  _sim.Update(20_ms);
  _IndexerMotor.IterateSim(_sim.GetAngularVelocity());
}
