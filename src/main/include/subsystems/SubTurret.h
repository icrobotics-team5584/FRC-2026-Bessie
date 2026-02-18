// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "utilities/AlertController.h"
#include "utilities/ICSparkMax.h"
#include "utilities/MechanismCircle2d.h"

#include <frc/Alert.h>
#include <frc/DutyCycleEncoder.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/geometry/Transform2d.h>
#include <frc/interpolation/TimeInterpolatableBuffer.h>
#include <frc/simulation/DCMotorSim.h>
#include <frc/simulation/EncoderSim.h>
#include <frc/smartdashboard/Mechanism2d.h>
#include <frc/smartdashboard/MechanismLigament2d.h>
#include <frc/system/plant/DCMotor.h>
#include <frc/system/plant/LinearSystemId.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/button/CommandXboxController.h>

#include <units/angle.h>

#include "Constants.h"


class SubTurret : public frc2::SubsystemBase {
 public:
  SubTurret();
  static SubTurret& GetInstance() {
    static SubTurret inst;
    return inst;
  }

  void SimulationPeriodic();

  units::degree_t GetTurretAngleCRT();
  units::degree_t GetTurretAngle();
  units::degree_t GetTurretAngleAtTime(units::second_t time);
  units::degree_t CalcOptimisedTurretAngle(units::degree_t angle);
  units::degree_t GetFieldRelativeTurretAngle();
  units::degree_t GetTurretTargetAngle();
    
  void SetTurretAngle(units::degree_t angle);
  void ZeroTurret();

  bool IsAtTarget();
  bool IsNotApproachingMax(std::function<units::millisecond_t()> time);

  frc2::CommandPtr SetTurretTargetAngle(std::function<units::degree_t()> angle,
    std::function<units::degrees_per_second_t()> robotAngVel);
  frc2::CommandPtr ZeroTurretCmd();

  static constexpr frc::Transform2d ROBOT_TO_TURRET = frc::Transform2d{-235_mm, 0_mm, 0_deg};

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
  ICSparkMax _turretMotor{canid::TURRET_MOTOR};
  rev::spark::SparkBaseConfig _turretMotorConfig;

  frc::Alert _turrethighTemperatureAlert{
    "Turret Motor High Temperature!", frc::Alert::AlertType::kWarning};
  frc::Alert _turretCurrentAlert{"Turret Motor Overcurrent!", frc::Alert::AlertType::kWarning};

  AlertController::MotorAlertConfig _turretAlertConfig{
    _turrethighTemperatureAlert, _turretCurrentAlert, 60_degC, 20_A};

  frc::DutyCycleEncoder _turretEncoder1{dio::TURRET_ENCODER_1};
  frc::DutyCycleEncoder _turretEncoder2{dio::TURRET_ENCODER_2};

  units::degree_t getEncoder1Degrees();
  units::degree_t getEncoder2Degrees();

  static constexpr frc::DCMotor MOTOR_MODEL = frc::DCMotor::NEO();
  static constexpr units::kilogram_square_meter_t MOI = 0.0001_kg_sq_m;

  frc::SimpleMotorFeedforward<units::turn> _robotRotVelFF{kS, kV, kA};

  const double encoder1ZeroOffset = 0.696408;
  const double encoder2ZeroOffset = 0.120609;
  const units::turn_t turretZeroOffset = -0.5_tr;

  units::degree_t POS_LIMIT = 360_deg;
  units::degree_t NEG_LIMIT = 0_deg;

  bool _hasZeroed = false;

  double P = 8.0;
  double I = 0.01;
  double D = 4;

  static constexpr units::volt_t kS = 0.15_V;
  static constexpr auto kV = 3.4_V * (1_s / 1_tr);
  static constexpr auto kA = 0_V * ((1_s * 1_s) / 1_tr);

  static constexpr double E1_TEETH = 21;
  static constexpr double E2_TEETH = 20;
  static constexpr double BIG_TEETH = 94;
  static constexpr double ENCODER1_RATIO = E1_TEETH / BIG_TEETH;
  static constexpr double ENCODER2_RATIO = E2_TEETH / BIG_TEETH;
  static constexpr double GEAR_RATIO = (48.0 / 12.0) * (94.0 / 10.0);

  static constexpr units::degree_t TOLARANCE = 2_deg;
  static constexpr units::hertz_t ENCODER_FREQUENCY = 975.6_Hz;
  // force set encoder frequency to avoid 1sec startup time

  frc::TimeInterpolatableBuffer<units::degree_t> _turretPos{1_s};

  // Sim
  frc::LinearSystem<2, 1, 2> _turretSystem =
    frc::LinearSystemId::DCMotorSystem(MOTOR_MODEL, MOI, GEAR_RATIO);
  frc::sim::DCMotorSim _turretSim{_turretSystem, MOTOR_MODEL};

  // mechanism2d
  frc::Mechanism2d _turretMech{0.25, 0.25};
  frc::MechanismRoot2d* _turretMechRoot = _turretMech.GetRoot("turretRoot", 0.125, 0.125);
  MechanismCircle2d _turretMechCircle{_turretMechRoot, "turretCircle", 0.05, 0_deg};
};
