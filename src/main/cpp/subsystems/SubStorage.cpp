// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubStorage.h"

#include <frc/Alert.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <units/current.h>
#include <utilities/Logger.h>

#include "Constants.h"

SubStorage::SubStorage() {
  _storageMotorConfig.SmartCurrentLimit(60);
  _storageMotor.OverwriteConfig(_storageMotorConfig);
  Logger::Log("Storage/Storage Motor", &_storageMotor);

  StorageEnterTrigger().OnTrue(frc2::cmd::RunOnce([this] { fuelCount++; }));
}

frc2::CommandPtr SubStorage::StorageOn() {
  return StartEnd([this] { _storageMotor.Set(1); }, [this] { _storageMotor.Set(0); });
};

frc2::CommandPtr SubStorage::StorageOff() {
  return RunOnce([this] { _storageMotor.Set(0); });
};

bool SubStorage::IsFull() {
  return _storagFullSensor.Get();
}

bool SubStorage::IsEmpty() {
  return !_storageEmptySensor.Get();
}

frc2::Trigger SubStorage::StorageEnterTrigger() {
  return frc2::Trigger([this] { return _storageEnterSensor.Get(); });
}

void SubStorage::CurrentHighTimer() {
  _storageHighCurrentTimer.Start();

  if (_storageHighCurrentTimer.Get() > 3_s) {
    storageCurrentAlert.Set(true);
  }
};

// This method will be called once per scheduler run
void SubStorage::Periodic() {
  frc::SmartDashboard::PutBoolean("Storage/IsFull", IsFull());
  frc::SmartDashboard::PutBoolean("Storage/IsEmpty", IsEmpty());
  frc::SmartDashboard::PutNumber("Storage/Fuel Count", fuelCount);

  units::ampere_t current = _storageMotor.GetOutputCurrent() * 1_A;
  frc::SmartDashboard::PutNumber("Storage/Current", current.value());
  if (current > 20_A) {
    SubStorage::CurrentHighTimer();
  } else {
    storageCurrentAlert.Set(false);
    _storageHighCurrentTimer.Reset();
  }

  units::celsius_t temperature = _storageMotor.GetTemperature();
  frc::SmartDashboard::PutNumber("Storage/Temperature", temperature.value());
  if (temperature > 60_degC) {
    highTemperatureAlert.Set(true);
  } else {
    highTemperatureAlert.Set(false);
  }
}

void SubStorage::SimulationPeriodic() {
  _sim.SetInputVoltage(_storageMotor.CalcSimVoltage());
  _sim.Update(20_ms);
  _storageMotor.IterateSim(_sim.GetAngularVelocity());
}
