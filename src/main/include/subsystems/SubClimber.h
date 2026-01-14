// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "utilities/ICSparkFlex.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Commands.h>
#include <frc2/command/SubsystemBase.h>

#include <rev/config/SparkFlexConfig.h>
#include <rev/config/SparkFlexConfigAccessor.h>

#include "Constants.h"

class SubClimber : public frc2::SubsystemBase {
 public:
  static SubClimber& GetInstance() {
    static SubClimber instance;
    return instance;
  }
  SubClimber();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  void SimulationPeriodic() override;

 private:
  /* place holder values */
  const double P = 0;
  const double I = 0;
  const double D = 0;
  const double GEAR_RATIO = 1 / 1; /* mechanism rotations per motor rotations*/

  ICSparkFlex _topClimbMotor{canid::CLIMBER_TOP};
  ICSparkFlex _botClimbMotor{canid::CLIMBER_BOT};
  rev::spark::SparkBaseConfig _topClimbMotorConfig;
  rev::spark::SparkBaseConfig _botClimbMotorConfig;
};
