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

  _outdexerMotorConfig.SmartCurrentLimit(60);
  _outdexerMotorConfig.Inverted(true);
  _outdexerMotor.OverwriteConfig(_outdexerMotorConfig);
  Logger::Log("Indexer/Indexer Out motor", &_outdexerMotor);
}


//Spindexer motor
frc2::CommandPtr SubIndexer::IndexerOn() {
  return StartEnd([this] { _indexerMotor.Set(1); }, [this] { _indexerMotor.Set(0); });
}

frc2::CommandPtr SubIndexer::IndexerOff() {
  return RunOnce([this] { _indexerMotor.Set(0); });
}

void SubIndexer::IndexerCurrentHighTimer() {
  _indexerHighCurrentTimer.Start();

  if (_indexerHighCurrentTimer.Get() > 3_s) {
    _indexerCurrentAlert.Set(true);
  }
}

//Indexer horizontal roller motor
frc2::CommandPtr SubIndexer::IndexerOutOn(){
  return RunOnce([this]{ _outdexerMotor.Set(1); });
}

frc2::CommandPtr SubIndexer::IndexerOutOff(){
  return RunOnce([this]{ _outdexerMotor.Set(0); });
}

void SubIndexer::IndexerOutCurrentHighTimer() {
  _outdexerHighCurrentTimer.Start();

  if (_outdexerHighCurrentTimer.Get() > 3_s) {
    _outdexerCurrentAlert.Set(true);
  }
};



// This method will be called once per scheduler run
void SubIndexer::Periodic() {
  units::ampere_t IndexerCurrent = _indexerMotor.GetOutputCurrent() * 1_A;
  Logger::Log("Indexer/Indexer Motor Current", IndexerCurrent);
  if (IndexerCurrent > 20_A) {
    IndexerCurrentHighTimer();
  } else {
    _indexerCurrentAlert.Set(false);
    _indexerHighCurrentTimer.Reset();
  }

  units::celsius_t IndexerTemp = _indexerMotor.GetTemperature();
  Logger::Log("Indexer/Indexer Motor Temperature", IndexerTemp);
  if (IndexerTemp > 60_degC) {
    _indexerHighTemperatureAlert.Set(true);
  } else {
    _indexerHighTemperatureAlert.Set(false);
  }


  units::ampere_t IndexerOutcurrent = _outdexerMotor.GetOutputCurrent() * 1_A;
  Logger::Log("Indexer/Indexer Motor Current", IndexerOutcurrent);
  if (IndexerOutcurrent > 20_A) {
    IndexerOutCurrentHighTimer();
  } else {
    _outdexerCurrentAlert.Set(false);
    _outdexerHighCurrentTimer.Reset();
  }

  units::celsius_t IndexerOutTemp = _outdexerMotor.GetTemperature();
  Logger::Log("Indexer/Indexer Motor Temperature", IndexerOutTemp);
  if (IndexerOutTemp > 60_degC) {
    _outdexerHighTemperatureAlert.Set(true);
  } else {
    _outdexerHighTemperatureAlert.Set(false);
  }
}

void SubIndexer::SimulationPeriodic() {
  _sim.SetInputVoltage(_indexerMotor.CalcSimVoltage());
  _sim.Update(20_ms);
  _indexerMotor.IterateSim(_sim.GetAngularVelocity());
}

frc2::CommandPtr SubIndexer::Index() {
  return StartEnd([this] { _indexerMotor.Set(0.5); _outdexerMotor.Set(0.5); }, [this] { _indexerMotor.Set(0); _outdexerMotor.Set(0);});
}


frc2::CommandPtr SubIndexer::NotIndex() {
  return RunOnce([this] { _indexerMotor.Set(0); _outdexerMotor.Set(0); });
}
