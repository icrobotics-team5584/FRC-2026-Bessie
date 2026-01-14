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
  units::ampere_t  GetLeftMotorCurrent();
  units::ampere_t  GetRightMotorCurrent();
  
  /* Commands */
  frc2::CommandPtr WaitUntilReset();
  frc2::CommandPtr ReadyClimber();
  frc2::CommandPtr ClimbL1();
  frc2::CommandPtr ClimbL2();
  frc2::CommandPtr ClimbL3();


 private:
  bool _hasreset = false;
  bool _resetting = false;

  /* place holder values */
  static constexpr units::turn_t LEFT_READY_TURNS = 0_deg;
  static constexpr units::turn_t RIGHT_READY_TURNS = 0_deg;
  static constexpr units::turn_t LEFT_L1_TURNS = 0_deg;
  static constexpr units::turn_t RIGHT_L1_TURNS = 0_deg;
  static constexpr units::turn_t LEFT_L2_TURNS = 0_deg;
  static constexpr units::turn_t RIGHT_L2_TURNS = 0_deg;
  static constexpr units::turn_t LEFT_L3_TURNS = 0_deg;
  static constexpr units::turn_t RIGHT_L3_TURNS = 0_deg;

  static constexpr units::turn_t TOLERANCE = 1_deg;
  static constexpr units::ampere_t ZEROING_CURRENT = 30_A;

  /* place holder values */
  const double P = 0;
  const double I = 0;
  const double D = 0;
  const double GEAR_RATIO = 1 / 1; /* mechanism rotations per motor rotations*/

  ICSparkFlex _leftClimbMotor{canid::CLIMBER_LEFT};
  ICSparkFlex _rightClimbMotor{canid::CLIMBER_RIGHT};
  rev::spark::SparkBaseConfig _leftClimbMotorConfig;
  rev::spark::SparkBaseConfig _rightClimbMotorConfig;
};
