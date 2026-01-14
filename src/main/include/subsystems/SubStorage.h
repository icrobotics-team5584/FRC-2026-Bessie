// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <utilities/IcSparkFlex.h>
#include "Constants.h"
#include "frc2/command/Commands.h"
#include "rev/config/SparkFlexConfig.h"
#include <frc/DigitalInput.h>


class SubStorage : public frc2::SubsystemBase {
 public:
 static SubStorage& GetInstance() {
    static SubStorage instance;
    return instance;
  }
  SubStorage();
  
  frc2::CommandPtr StorageOn();
  frc2::CommandPtr StorageOff();

 

  bool IsFull();
  bool IsEmpty();
  bool CheckSensorHigher();
  bool CheckSensorLower();

 
  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
  ICSparkFlex _storageMotor{canid::STORAGE};
  rev::spark::SparkFlexConfig _storageMotorConfig;

  frc::DigitalInput _storageSensorHigher {dio::STORAGE_SENSOR_HIGHER};
  frc::DigitalInput _storageSensorLower {dio::STORAGE_SENSOR_LOWER};
};
