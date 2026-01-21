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

  _indexerOutMotorConfig.SmartCurrentLimit(60);
  _indexerOutMotor.OverwriteConfig(_indexerOutMotorConfig);
  Logger::Log("Indexer/Indexer Out motor", &_indexerOutMotor);
}


//Spindexer motor
frc2::CommandPtr SubIndexer::IndexerOn() {
  return StartEnd([this] { _indexerMotor.Set(1); }, [this] { _indexerMotor.Set(0); });
};

frc2::CommandPtr SubIndexer::IndexerOff() {
  return RunOnce([this] { _indexerMotor.Set(0); });
};

void SubIndexer::IndexerCurrentHighTimer() {
  _indexerHighCurrentTimer.Start();

  if (_indexerHighCurrentTimer.Get() > 3_s) {
    IndexerCurrentAlert.Set(true);
  }
};

//Indexer horizontal roller motor
frc2::CommandPtr SubIndexer::IndexerOutOn(){
  return RunOnce([this]{ _indexerOutMotor.Set(1); });
}

frc2::CommandPtr SubIndexer::IndexerOutOff(){
  return RunOnce([this]{ _indexerOutMotor.Set(0); });
}

void SubIndexer::IndexerOutCurrentHighTimer() {
  _indexerOutHighCurrentTimer.Start();

  if (_indexerOutHighCurrentTimer.Get() > 3_s) {
    IndexerOutCurrentAlert.Set(true);
  }
};



// This method will be called once per scheduler run
void SubIndexer::Periodic() {
  units::ampere_t IndexerCurrent = _indexerMotor.GetOutputCurrent() * 1_A;
  Logger::Log("Indexer/Indexer Motor Current", IndexerCurrent);
  if (IndexerCurrent > 20_A) {
    IndexerCurrentHighTimer();
  } else {
    IndexerCurrentAlert.Set(false);
    _indexerHighCurrentTimer.Reset();
  }

  units::celsius_t IndexerTemp = _indexerMotor.GetTemperature();
  Logger::Log("Indexer/Indexer Motor Temperature", IndexerTemp);
  if (IndexerTemp > 60_degC) {
    IndexerHighTemperatureAlert.Set(true);
  } else {
    IndexerHighTemperatureAlert.Set(false);
  }


  units::ampere_t IndexerOutcurrent = _indexerOutMotor.GetOutputCurrent() * 1_A;
  Logger::Log("Indexer/Indexer Motor Current", IndexerOutcurrent);
  if (IndexerOutcurrent > 20_A) {
    IndexerOutCurrentHighTimer();
  } else {
    IndexerOutCurrentAlert.Set(false);
    _indexerOutHighCurrentTimer.Reset();
  }

  units::celsius_t IndexerOutTemp = _indexerOutMotor.GetTemperature();
  Logger::Log("Indexer/Indexer Motor Temperature", IndexerOutcurrent);
  if (IndexerOutcurrent > 60_degC) {
    IndexerOutHighTemperatureAlert.Set(true);
  } else {
    IndexerOutHighTemperatureAlert.Set(false);
  }
}

void SubIndexer::SimulationPeriodic() {
  _sim.SetInputVoltage(_indexerMotor.CalcSimVoltage());
  _sim.Update(20_ms);
  _indexerMotor.IterateSim(_sim.GetAngularVelocity());
}
