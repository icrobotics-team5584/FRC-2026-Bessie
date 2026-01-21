// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "utilities/ICSparkFlex.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/RobotBase.h>
#include <frc2/command/Commands.h>
#include <frc2/command/SubsystemBase.h>

#include <rev/config/SparkFlexConfig.h>
#include <rev/config/SparkFlexConfigAccessor.h>

#include <units/angle.h>
#include <units/current.h>

#include "Constants.h"

class SubClimber : public frc2::SubsystemBase {
 public:
  static SubClimber& GetInstance() {
    static SubClimber instance;
    return instance;
  }
  SubClimber();

  /* Will be called periodically whenever the CommandScheduler runs. */
  void Periodic() override;
  void SimulationPeriodic() override;

  /* Instaneous */
  void SetBrakeMode(bool isBrake);
  bool IsAtTarget();
  units::ampere_t GetMotorCurrent();
  
  /* Commands */
  frc2::CommandPtr StowClimber();
  frc2::CommandPtr ReadyClimber();
  frc2::CommandPtr ClimbL1();

  frc2::CommandPtr RunCurrentZeroingSequence();


 private:
  bool _hasZeroed = false;
  bool _zeroing = false;

  /* place holder values */
  static constexpr units::degree_t _STOW_TURNS = 0_deg;
  static constexpr units::degree_t _READY_TURNS = 0_deg;
  static constexpr units::degree_t _L1_TURNS = 0_deg;

  static constexpr units::degree_t _TOLERANCE = 1_deg;
  static constexpr units::ampere_t _ZEROING_CURRENT = 30_A;

  /* place holder values */
  static constexpr double _P = 0;
  static constexpr double _I = 0;
  static constexpr double _D = 0;
  static constexpr double _GEAR_RATIO = 1;

  ICSparkFlex _climberMotor{canid::CLIMBER};
  rev::spark::SparkBaseConfig _climberMotorConfig;
};
