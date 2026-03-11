// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubIndexer.h"

#include "utilities/RobotVisualisation.h"

#include <units/current.h>
#include <utilities/Logger.h>

SubIndexer::SubIndexer() {
  _indexerMotorConfig.SmartCurrentLimit(60);
  _indexerMotor.OverwriteConfig(_indexerMotorConfig);
  Logger::Log("Indexer/Indexer Motor", &_indexerMotor);

  _indexerAlertConfig = std::make_shared<AlertController::AlertConfig>(
    "Indexer Motor", 60_degC, 20_A);
   AlertController::RegisterAlertConfig(_indexerAlertConfig);
}

// Spindexer motor
frc2::CommandPtr SubIndexer::IndexerOn() {
  return StartEnd([this] { _indexerMotor.Set(0.3); }, [this] { _indexerMotor.Set(0); });
}

frc2::CommandPtr SubIndexer::IndexerOff() {
  return RunOnce([this] { _indexerMotor.Set(0); });
}

// This method will be called once per scheduler run
void SubIndexer::Periodic() {
  auto loopStart = frc::GetTime();

  units::ampere_t IndexerCurrent = _indexerMotor.GetOutputCurrent() * 1_A;
  Logger::Log("Indexer/Indexer Motor Current", IndexerCurrent);

  units::celsius_t IndexerTemp = _indexerMotor.GetTemperature();
  Logger::Log("Indexer/Indexer Motor Temperature", IndexerTemp);

  AlertController::MotorTelemetryConfig telemetryConfig{IndexerTemp, IndexerCurrent};
  AlertController::UpdateAllAlerts(*_indexerAlertConfig, telemetryConfig);

  RobotVisualisation::GetInstance()._indexerMechCircle.SetAngle(_indexerMotor.GetPosition());
  
  Logger::Log("Indexer/Loop Time", (frc::GetTime() - loopStart));
}

void SubIndexer::SimulationPeriodic() {
  _sim.SetInputVoltage(_indexerMotor.CalcSimVoltage());
  _sim.Update(20_ms);
  _indexerMotor.IterateSim(_sim.GetAngularVelocity());
}

frc2::CommandPtr SubIndexer::Index() {
  return StartEnd(
    [this] {
      _indexerMotor.Set(0.3);
    },
    [this] {
      _indexerMotor.Set(0);
    });
}

frc2::CommandPtr SubIndexer::StopIndex() {
  return RunOnce([this] {
    _indexerMotor.Set(0);
  });
}
