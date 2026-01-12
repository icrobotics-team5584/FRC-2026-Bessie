// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include "utilities/ICSparkMax.h"
#include "Constants.h"
#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>

#include <frc/simulation/FlywheelSim.h>
#include <frc/system/plant/DCMotor.h>
#include <frc/system/plant/LinearSystemId.h>

class SubShooter : public frc2::SubsystemBase {
 public:
  SubShooter();
  static SubShooter& GetInstance() {
    static SubShooter inst;
    return inst;
  }

  void SimulationPeriodic();

  frc2::CommandPtr SetShooterSpeed(units::turns_per_second_t speed);
  frc2::CommandPtr SpinUpShooter();
  frc2::CommandPtr StopShooter();
  bool IsAtSpeed();

  

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
  double GEAR_RATIO = 1.0/2;
  ICSparkMax _shooterMotor1{canid::SHOOTER_MOTOR_1, 30_A};
  ICSparkMax _shooterMotor2{canid::SHOOTER_MOTOR_2, 30_A};
  static constexpr units::kilogram_square_meter_t MOI = 0.05_kg_sq_m;
  static constexpr frc::DCMotor MOTOR_MODEL = frc::DCMotor::NEO();

  double P = 1.0;
  double I = 0;
  double D = 0;
  units::turns_per_second_t SHOOTER_SPEED = 1_tps;

  rev::spark::SparkBaseConfig _shooterMotor1Config;
  rev::spark::SparkBaseConfig _shooterMotor2Config;

  //Sim
  frc::LinearSystem<1,1,1> _flywheelSystem = frc::LinearSystemId::FlywheelSystem(MOTOR_MODEL, MOI, GEAR_RATIO);
  frc::sim::FlywheelSim _flywheelSim{_flywheelSystem, MOTOR_MODEL};
};
