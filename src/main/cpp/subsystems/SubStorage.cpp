// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubStorage.h"

#include <frc/smartdashboard/SmartDashboard.h>

#include <units/current.h>
#include <utilities/Logger.h>

#include "Constants.h"

SubStorage::SubStorage() {
  _storageMotorConfig.SmartCurrentLimit(60);
  _storageMotor.OverwriteConfig(_storageMotorConfig);
  Logger::Log("Storage/Storage Motor", &_storageMotor);
  // Logger::Log("Storage/Storage Sensor Higher", &_storageSensorHigher);
  // Logger::Log("Storage/Storage Sensor Lower", &_storageSensorLower);
}

frc2::CommandPtr SubStorage::StorageOn() {
  return StartEnd(
    [this] {
      _storageMotor.Set(1);
    },
    [this] { _storageMotor.Set(0); });
};

frc2::CommandPtr SubStorage::StorageOff() {
  return RunOnce([this] { _storageMotor.Set(0); });
};

bool SubStorage::CheckSensorHigher() {
  // return true;
  return !_storageSensorHigher.Get();
}

bool SubStorage::CheckSensorLower() {
  // return true;
  return !_storageSensorLower.Get();
}

bool SubStorage::IsFull() {
  return _storageSensorHigher.Get();
}

bool SubStorage::IsEmpty() {
  return !_storageSensorLower.Get();
}

// This method will be called once per scheduler run
void SubStorage::Periodic() {
  frc::SmartDashboard::PutBoolean("Storage/IsFull", IsFull());
  frc::SmartDashboard::PutBoolean("Storage/IsEmpty", IsEmpty());

  // if (()) {
  //  StoreCurrentAmount();
  // }
}
