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

#include <frc/simulation/DCMotorSim.h>
#include <frc/simulation/SingleJointedArmSim.h>
#include <frc/system/plant/DCMotor.h>
#include <frc/system/plant/LinearSystemId.h>
#include <frc/smartdashboard/Mechanism2d.h>
#include <frc/smartdashboard/MechanismLigament2d.h>
#include "utilities/MechanismCircle2d.h"

class SubHood : public frc2::SubsystemBase {
 public:
  SubHood();
  static SubHood& GetInstance() {
    static SubHood inst;
    return inst;
  }

  void SimulationPeriodic();

  bool HoodCurrentCheck(); 

  wpi::interpolating_map<units::meter_t, units::degree_t> _pitchTable;
  
  units::ampere_t GetHoodMotorCurrent();
  units::degree_t GetAngleFromDistance(units::meter_t distance);

  frc2::CommandPtr ManualHoodDown(); 
  frc2::CommandPtr StowHood(); 
  frc2::CommandPtr ZeroHood();
  frc2::CommandPtr SetHoodPositionTarget(units::degree_t angle);
  
  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:

  double P = 1.0;
  double I = 0.0;
  double D = 0.0;

  units::ampere_t zeroingCurrentLimit = 15_A;

  static constexpr units::degree_t UPPER_LIMIT = 35.0_deg;
  static constexpr units::degree_t LOWER_LIMIT = 12.5_deg;
  static constexpr bool SIMULATE_GRAVITY = true;
  static constexpr units::degree_t STARTING_ANGLE = 13_deg;
  static constexpr units::degree_t STOW_ANGLE = 12.5_deg;
  static constexpr double GEAR_RATIO = (42.0/8.0) * (400.0/24.0);
  static constexpr units::centimeter_t ARM_LENGTH = 20_cm;

  bool _zeroing = false;
  bool _hasZeroed = false;

  ICSparkMax _hoodMotor{canid::HOOD_MOTOR};
  rev::spark::SparkBaseConfig _hoodMotorConfig;

  static constexpr frc::DCMotor MOTOR_MODEL = frc::DCMotor::NEO550();
  static constexpr units::kilogram_square_meter_t MOI = 0.0001_kg_sq_m;

  //Sim
  frc::LinearSystem<2,1,2> _hoodSystem = frc::LinearSystemId::SingleJointedArmSystem(MOTOR_MODEL, MOI, GEAR_RATIO);
  frc::sim::SingleJointedArmSim _hoodSim{_hoodSystem, MOTOR_MODEL, GEAR_RATIO, ARM_LENGTH, 
    LOWER_LIMIT, UPPER_LIMIT, SIMULATE_GRAVITY, STARTING_ANGLE};

  //mechanism2d
  frc::Mechanism2d _hoodMech{0.25, 0.25};
  frc::MechanismRoot2d* _hoodMechRoot = _hoodMech.GetRoot("hoodRoot", 0.125, 0.125);
  MechanismCircle2d _hoodMechCircle{_hoodMechRoot, "hoodCircle", 0.05, 0_deg};
};