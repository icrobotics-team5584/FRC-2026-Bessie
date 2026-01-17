// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include "utilities/ICSparkMax.h"
#include "Constants.h"
#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>
#include <frc/DutyCycleEncoder.h>
#include <units/angle.h>
#include <frc2/command/button/CommandXboxController.h>

#include <frc/simulation/DCMotorSim.h>
#include <frc/system/plant/DCMotor.h>
#include <frc/system/plant/LinearSystemId.h>
#include <frc/smartdashboard/Mechanism2d.h>
#include <frc/smartdashboard/MechanismLigament2d.h>
#include "utilities/MechanismCircle2d.h"
#include <frc/simulation/EncoderSim.h>


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
  units::degree_t CalcOptimisedTurretAngle(units::degree_t angle);
  
  void SetTurretAngle(units::degree_t angle);
  void ZeroTurret();

  frc2::CommandPtr SetTurretTargetAngle(std::function<units::degree_t()> angle);
  frc2::CommandPtr ZeroTurretCmd();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
  ICSparkMax _turretMotor{canid::TURRET_MOTOR};
  rev::spark::SparkBaseConfig _turretMotorConfig;

  frc::DutyCycleEncoder _turretEncoder1{dio::TURRET_ENCODER_1};
  frc::DutyCycleEncoder _turretEncoder2{dio::TURRET_ENCODER_2};

  units::degree_t getEncoder1Degrees();
  units::degree_t getEncoder2Degrees();

  static constexpr frc::DCMotor MOTOR_MODEL = frc::DCMotor::NEO();
  static constexpr units::kilogram_square_meter_t MOI = 0.0001_kg_sq_m;

  const double encoder1ZeroOffset = 0.998531;
  const double encoder2ZeroOffset = 0.339566;

  units::degree_t POS_LIMIT = 270_deg;
  units::degree_t NEG_LIMIT = -270_deg;

  bool _hasZeroed = false;

  double P = 2.0;
  double I = 0;
  double D = 0;
  
  static constexpr double E1_TEETH = 21;
  static constexpr double E2_TEETH = 20;
  static constexpr double BIG_TEETH = 94;
  static constexpr double ENCODER1_RATIO = E1_TEETH/BIG_TEETH;
  static constexpr double ENCODER2_RATIO = E2_TEETH/BIG_TEETH;
  static constexpr double GEAR_RATIO = (48.0/12.0) * (94.0/10.0);


  static constexpr units::hertz_t ENCODER_FREQUENCY = 975.6_Hz; 
  //force set encoder frequency to avoid 1sec startup time

  //Sim
  frc::LinearSystem<2,1,2> _turretSystem = frc::LinearSystemId::DCMotorSystem(MOTOR_MODEL, MOI, GEAR_RATIO);
  frc::sim::DCMotorSim _turretSim{_turretSystem, MOTOR_MODEL};

  //mechanism2d
  frc::Mechanism2d _turretMech{0.25, 0.25};
  frc::MechanismRoot2d* _turretMechRoot = _turretMech.GetRoot("turretRoot", 0.125, 0.125);
  MechanismCircle2d _turretMechCircle{_turretMechRoot, "turretCircle", 0.05, 0_deg};
};
