// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "utilities/AlertController.h"
#include "utilities/ICSparkMax.h"
#include "utilities/MechanismCircle2d.h"

#include <frc/Alert.h>
#include <frc/simulation/DCMotorSim.h>
#include <frc/simulation/SingleJointedArmSim.h>
#include <frc/smartdashboard/Mechanism2d.h>
#include <frc/smartdashboard/MechanismLigament2d.h>
#include <frc/system/plant/DCMotor.h>
#include <frc/system/plant/LinearSystemId.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>
#include <frc2/command/SubsystemBase.h>

#include <units/angle.h>
#include <wpi/interpolating_map.h>

#include "Constants.h"


class SubHood : public frc2::SubsystemBase {
 public:
  SubHood();
  static SubHood& GetInstance() {
    static SubHood inst;
    return inst;
  }

  void SimulationPeriodic();

  bool HoodCurrentCheck();
  bool HoodIsAtTarget();

  units::ampere_t GetHoodMotorCurrent();

  frc2::CommandPtr ManualHoodDown();
  frc2::CommandPtr StowHood();
  frc2::CommandPtr ZeroHood();
  frc2::CommandPtr SetHoodPositionTarget(std::function<units::degree_t()> angle);
  frc2::CommandPtr SetHoodPositionTargetFromDist(std::function<units::meter_t()> distanceToTarget);
  frc2::CommandPtr MoveHoodUp1Degree();
  frc2::CommandPtr MoveHoodDown1Degree();

  static constexpr units::degree_t PASSING_ANGLE = 37_deg;

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
  double P = 16.0;
  double I = 0.0;
  double D = 8.0;
  double S = 0.6;

  units::ampere_t zeroingCurrentLimit = 22_A;

  static constexpr units::degree_t UPPER_LIMIT = 37.5_deg;
  static constexpr units::degree_t LOWER_LIMIT = 16.5_deg;
  static constexpr bool SIMULATE_GRAVITY = true;
  static constexpr units::degree_t STARTING_ANGLE = 13_deg;
  static constexpr units::degree_t STOW_ANGLE = 12.5_deg;
  static constexpr double GEAR_RATIO = (56.0 / 8.0) * (370.0 / 34.0);
  static constexpr units::centimeter_t ARM_LENGTH = 20_cm;
  static constexpr units::degree_t TOLARANCE = 0.5_deg;

  bool _zeroing = false;
  bool _hasZeroed = false;

  ICSparkMax _hoodMotor{canid::HOOD_MOTOR};
  rev::spark::SparkBaseConfig _hoodMotorConfig;

  frc::Alert _hoodhighTemperatureAlert{
    "Hood Motor High Temperature!", frc::Alert::AlertType::kWarning};
  frc::Alert _hoodCurrentAlert{"Hood Motor Overcurrent!", frc::Alert::AlertType::kWarning};

  AlertController::MotorAlertConfig _hoodAlertConfig{
    _hoodhighTemperatureAlert, _hoodCurrentAlert, 60_degC, 20_A};

  wpi::interpolating_map<units::meter_t, units::degree_t> _hoodPitchTable;

  static constexpr frc::DCMotor MOTOR_MODEL = frc::DCMotor::NEO550();
  static constexpr units::kilogram_square_meter_t MOI = 0.0001_kg_sq_m;

  // Sim
  frc::LinearSystem<2, 1, 2> _hoodSystem =
    frc::LinearSystemId::SingleJointedArmSystem(MOTOR_MODEL, MOI, GEAR_RATIO);
  frc::sim::SingleJointedArmSim _hoodSim{_hoodSystem, MOTOR_MODEL, GEAR_RATIO, ARM_LENGTH,
    LOWER_LIMIT, UPPER_LIMIT, SIMULATE_GRAVITY, STARTING_ANGLE};

  // mechanism2d
  frc::Mechanism2d _hoodMech{0.25, 0.25};
  frc::MechanismRoot2d* _hoodMechRoot = _hoodMech.GetRoot("hoodRoot", 0.125, 0.125);
  MechanismCircle2d _hoodMechCircle{_hoodMechRoot, "hoodCircle", 0.05, 0_deg};
};
