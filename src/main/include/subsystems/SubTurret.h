// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include "utilities/ICSparkMax.h"
#include "Constants.h"
#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>
#include <rev/SparkAbsoluteEncoder.h>

#include <frc/simulation/FlywheelSim.h>
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

  double GetTurretAngle(double encoder1, double encoder2);
  frc2::CommandPtr SetTurretAngle(units::degree_t angle);



  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
  ICSparkMax _turretMotor{canid::TURRET_MOTOR};
  rev::spark::SparkBaseConfig _turretMotorConfig;

  // rev::spark::SparkAbsoluteEncoder _turretEncoder1{dio::ENCODER_1A};
  // rev::spark::SparkAbsoluteEncoder _turretEncoder2{dio::ENCODER_2A, dio::ENCODER_2B};

  static constexpr frc::DCMotor MOTOR_MODEL = frc::DCMotor::NEO();
  static constexpr units::kilogram_square_meter_t MOI = 0.02_kg_sq_m;

  double P = 1.0;
  double I = 0;
  double D = 0;
  
  static constexpr double ENCODER1_RATIO = 20.0/94.0;
  static constexpr double ENCODER2_RATIO = 21.0/94.0;
  static constexpr double GEAR_RATIO = (12.0/48.0) * (10.0/94.0);

  //Sim
  frc::LinearSystem<1,1,1> _turretSystem = frc::LinearSystemId::FlywheelSystem(MOTOR_MODEL, MOI, GEAR_RATIO);
  frc::sim::FlywheelSim _turretSim{_turretSystem, MOTOR_MODEL};

  //mechanism2d
  frc::Mechanism2d _turretMech{0.25, 0.25};
  frc::MechanismRoot2d* _turretMechRoot = _turretMech.GetRoot("turretRoot", 0.125, 0.125);
  MechanismCircle2d _turretMechCircle{_turretMechRoot, "turretTopRoller", 0.05, 0_deg};
};
