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

  units::degree_t GetTurretAngle();
  frc2::CommandPtr SetTurretTargetAngle(units::degree_t angle);
  void SetTurretAngle(units::degree_t angle);
  void ZeroTurret();
  frc2::CommandPtr ZeroTurretCmd();
  frc2::CommandPtr zeroEncoders();
  void SetTurretTarget(units::degree_t angle);


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

  // double E1initial;
  // double E2initial;

  double E1initial = 0.998531;
  double E2initial = 0.339566;

  double POS_LIMIT = 90;
  double NEG_LIMIT = -90;

  bool _hasReset = false;

  double P = 2.0;
  double I = 0;
  double D = 0;
  double F = 1.0;
  
  static constexpr double ENCODER1_RATIO = 21.0/94.0;
  static constexpr double ENCODER2_RATIO = 20.0/94.0;
  static constexpr double GEAR_RATIO = (48.0/12.0) * (94.0/10.0);

  static constexpr double E1_TEETH = 21;
  static constexpr double E2_TEETH = 20;
  static constexpr double BIG_TOOTH = 94;

  //Sim
  frc::LinearSystem<2,1,2> _turretSystem = frc::LinearSystemId::DCMotorSystem(MOTOR_MODEL, MOI, GEAR_RATIO);
  frc::sim::DCMotorSim _turretSim{_turretSystem, MOTOR_MODEL};

  //mechanism2d
  frc::Mechanism2d _turretMech{0.25, 0.25};
  frc::MechanismRoot2d* _turretMechRoot = _turretMech.GetRoot("turretRoot", 0.125, 0.125);
  MechanismCircle2d _turretMechCircle{_turretMechRoot, "turretTopRoller", 0.05, 90_deg};
};
