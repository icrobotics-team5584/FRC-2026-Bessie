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

#include <units/angle.h>
#include <units/current.h>

#include "Constants.h"
#include <frc/simulation/ElevatorSim.h>

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
  frc2::CommandPtr ClimbToggle();

  frc2::CommandPtr ManualClimberUp();
  frc2::CommandPtr ManualClimberDown();

  frc2::CommandPtr RunCurrentZeroingSequence();


 private:
  bool _hasZeroed = false;
  bool _zeroing = false;

  /* place holder values */
  static constexpr units::degree_t _STOW_TURNS = 0_deg;
  static constexpr units::degree_t _READY_TURNS = 20_deg;
  static constexpr units::degree_t _L1_TURNS = 40_deg;

  static constexpr units::degree_t _TOLERANCE = 1_deg;
  static constexpr units::ampere_t _ZEROING_CURRENT = 30_A;

  /* place holder values */
  static constexpr double _P = 50;
  static constexpr double _I = 0;
  static constexpr double _D = 0;
  static constexpr double _GEAR_RATIO = 200;

  //sim constants (these are currently very arbitrary)
  static constexpr units::kilogram_t _CARRIAGE_MASS = 50_kg;
  static constexpr units::meter_t _DRUM_RADIUS = 1.5_cm;
  static constexpr units::meter_t _DRUM_CIRCUMFERENCE = _DRUM_RADIUS * 2 * std::numbers::pi;
  static constexpr units::meter_t _MIN_HEIGHT = 0_m;
  static constexpr units::meter_t _MAX_HEIGHT = 0.3_m;
  static constexpr units::meter_t _START_HEIGHT = 0_m;

  ICSparkFlex _climberMotor{canid::CLIMBER};
  rev::spark::SparkBaseConfig _climberMotorConfig;

  //sim
  frc::sim::ElevatorSim _climberSim{frc::DCMotor::NeoVortex(1), _GEAR_RATIO, _CARRIAGE_MASS,
    _DRUM_RADIUS, _MIN_HEIGHT, _MAX_HEIGHT, true, _START_HEIGHT};
};
