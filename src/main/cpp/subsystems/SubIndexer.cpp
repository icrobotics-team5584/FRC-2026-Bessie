// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubIndexer.h"

#include <units/current.h>
#include <utilities/Logger.h>

SubIndexer::SubIndexer() {
  _indexerMotorConfig.SmartCurrentLimit(60);
  _indexerMotor.OverwriteConfig(_indexerMotorConfig);
  Logger::Log("Indexer/Indexer Motor", &_indexerMotor);
}

frc2::CommandPtr SubIndexer::IndexerOn() {
  return StartEnd([this] { _indexerMotor.Set(1); }, [this] { _indexerMotor.Set(0); });
};

frc2::CommandPtr SubIndexer::IndexerOff() {
  return RunOnce([this] { _indexerMotor.Set(0); });
};

void SubIndexer::CurrentHighTimer() {
  _indexerHighCurrentTimer.Start();

  if (_indexerHighCurrentTimer.Get() > 3_s) {
    IndexerCurrentAlert.Set(true);
  }
};

// This method will be called once per scheduler run
void SubIndexer::Periodic() {
  units::ampere_t current = _indexerMotor.GetOutputCurrent() * 1_A;
  Logger::Log("Indexer/Indexer Motor Current", current);
  if (current > 20_A) {
    CurrentHighTimer();
  } else {
    IndexerCurrentAlert.Set(false);
    _indexerHighCurrentTimer.Reset();
  }

  units::celsius_t temperature = _indexerMotor.GetTemperature();
  Logger::Log("Indexer/Indexer Motor Temperature", temperature);
  if (temperature > 60_degC) {
    highTempuratureAlert.Set(true);
  } else {
    highTempuratureAlert.Set(false);
  }
}

void SubIndexer::SimulationPeriodic() {
  _sim.SetInputVoltage(_indexerMotor.CalcSimVoltage());
  _sim.Update(20_ms);
  _indexerMotor.IterateSim(_sim.GetAngularVelocity());
}
