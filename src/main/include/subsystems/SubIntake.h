// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "utilities/ICSparkFlex.h"

#include <frc2/command/SubsystemBase.h>

#include "Constants.h"
#include "frc2/command/Commands.h"
#include "frc2/command/SubsystemBase.h"
#include "rev/config/SparkFlexConfig.h"
#include "rev/config/SparkFlexConfigAccessor.h"

class SubIntake : public frc2::SubsystemBase {
 public:
  static SubIntake& GetInstance() {
    static SubIntake instance;
    return instance;
  }
  SubIntake();

  frc2::CommandPtr IntakeOn();
  frc2::CommandPtr IntakeOff();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
  ICSparkFlex _intakeMotor{canid::INTAKE};
  rev::spark::SparkFlexConfig _intakeMotorConfig;
};
