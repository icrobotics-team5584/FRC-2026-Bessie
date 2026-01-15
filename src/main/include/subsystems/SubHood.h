// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>

#include "utilities/ICSparkMax.h"
#include "Constants.h"
#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>
#include <units/angle.h>
#include <wpi/interpolating_map.h>

class SubHood : public frc2::SubsystemBase {
 public:
  SubHood();
  static SubHood& GetInstance() {
    static SubHood inst;
    return inst;
  }

  void SimulationPeriodic();

  units::degree_t GetAngleFromDistance(units::meter_t distance);

  frc2::CommandPtr SetHoodPosition(units::degree_t angle);
  frc2::CommandPtr ZeroHood();
  frc2::CommandPtr HoodResetCheck(); 
  units::ampere_t GetHoodMotorCurrent();
  frc2::CommandPtr ManualHoodDown(); 
  frc2::CommandPtr StowHood(); 
  frc2::CommandPtr PivotFromVision(std::function<units::meter_t()> distance);
  
  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:

  double P = 0.0;
  double I = 0.0;
  double D = 0.0;

  units::ampere_t zeroingCurrentLimit = 10_A;

  static constexpr units::degree_t UPPER_LIMIT = 35.0_deg;
  static constexpr units::degree_t LOWER_LIMIT = 12.5_deg;

  bool _resetting;
  bool _hasreset;

  units::turn_t STOW_TURNS = 0_tr;

  wpi::interpolating_map<units::meter_t, units::degree_t> _pitchTable;

  double GEAR_RATIO = (8.0/42.0) * (24.0/400.0);
  ICSparkMax _hoodMotor{canid::HOOD_MOTOR};
  rev::spark::SparkBaseConfig _hoodMotorConfig;
};
