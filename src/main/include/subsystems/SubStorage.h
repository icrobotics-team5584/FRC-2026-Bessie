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
#include <frc2/command/button/Trigger.h>


class SubStorage : public frc2::SubsystemBase {
 public:
 static SubStorage& GetInstance() {
    static SubStorage instance;
    return instance;
  }
  SubStorage();
  
  frc2::CommandPtr StorageOn();
  frc2::CommandPtr StorageOff();


  frc2::Trigger StorageEnterTrigger();
  
  bool IsFull();
  bool IsEmpty();
  bool CheckSensorHigher();
  bool CheckSensorLower();

 
  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
  int fuelCount = 0;
  ICSparkFlex _storageMotor{canid::STORAGE};
  rev::spark::SparkFlexConfig _storageMotorConfig;

  frc::DigitalInput _storagFullSensor {dio::STORAGE_SENSOR_FULL};
  frc::DigitalInput _storageEmptySensor {dio::STORAGE_SENSOR_EMPTY};
  frc::DigitalInput _storageEnterSensor {dio::STORAGE_SENSOR_ENTER};
};
