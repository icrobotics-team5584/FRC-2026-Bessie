// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "utilities/ICSparkFlex.h"
#include <frc2/command/SubsystemBase.h>
#include "frc2/command/Commands.h"
#include <Constants.h>
#include "rev/config/SparkFlexConfig.h"

class SubIndexor : public frc2::SubsystemBase {
 public:
 static SubIndexor& GetInstance() {
    static SubIndexor instance;
    return instance;
  }
  SubIndexor();

  frc2::CommandPtr IndexorOn();
  frc2::CommandPtr IndexorOff();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
    ICSparkFlex _indexorMotor{canid::INDEXOR};
  rev::spark::SparkFlexConfig _indexorMotorConfig;
};
