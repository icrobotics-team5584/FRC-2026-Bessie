// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include "Constants.h"
#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>

#include <frc/simulation/FlywheelSim.h>
#include <frc/system/plant/DCMotor.h>
#include <frc/system/plant/LinearSystemId.h>
#include <frc/smartdashboard/Mechanism2d.h>
#include <frc/smartdashboard/MechanismLigament2d.h>
#include "utilities/MechanismCircle2d.h"

class SubShooter : public frc2::SubsystemBase {
 public:
  SubShooter();
  static SubShooter& GetInstance() {
    static SubShooter inst;
    return inst;
  }

  void SimulationPeriodic();

  frc2::CommandPtr SetShooterTarget(units::turns_per_second_t speed);
  frc2::CommandPtr SetShooterTarget(std::function<units::turns_per_second_t()> speed);
  frc2::CommandPtr SetShooterTarget(std::function<units::meters_per_second_t()> speed);
  frc2::CommandPtr SpinUpShooter();
  frc2::CommandPtr StopShooter();
  
  bool IsAtSpeed();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
  ctre::phoenix6::hardware::TalonFX _shooterMotor1{canid::SHOOTER_MOTOR_1};
  ctre::phoenix6::hardware::TalonFX _shooterMotor2{canid::SHOOTER_MOTOR_2};
  static constexpr units::kilogram_square_meter_t MOI = 0.05_kg_sq_m;
  static constexpr frc::DCMotor MOTOR_MODEL = frc::DCMotor::KrakenX60FOC();
  static constexpr double GEAR_RATIO = 1.0;

  double P = 0.4;
  double I = 0;
  double D = 0;
  double V = 0.12;

  ctre::phoenix6::configs::TalonFXConfiguration _shooterMotorConfig;
  ctre::phoenix6::controls::VelocityVoltage _flywheelTargetVelocity{0_tps};

  units::meter_t wheelDiameter = 0.24_m;
  double energyEff = 0.5;

  //Sim
  frc::LinearSystem<1,1,1> _leftFlywheelSystem = frc::LinearSystemId::FlywheelSystem(MOTOR_MODEL, MOI, GEAR_RATIO);
  frc::sim::FlywheelSim _leftFlywheelSim{_leftFlywheelSystem, MOTOR_MODEL};

  frc::LinearSystem<1,1,1> _rightFlywheelSystem = frc::LinearSystemId::FlywheelSystem(MOTOR_MODEL, MOI, GEAR_RATIO);
  frc::sim::FlywheelSim _rightFlywheelSim{_rightFlywheelSystem, MOTOR_MODEL};

  //mechanism2d
  frc::Mechanism2d _shooterMech{0.25, 0.25};
  frc::MechanismRoot2d* _shooterMechRoot = _shooterMech.GetRoot("shooterRoot", 0.125, 0.125);
  frc::MechanismLigament2d *_shooterMechUpperConnector =
    _shooterMechRoot->Append<frc::MechanismLigament2d>("shooterUpperConnector", 0.05, 90_deg, 0);
  MechanismCircle2d _shooterMechTopRoller{_shooterMechUpperConnector, "shooterTopRoller", 0.025, 0_deg};
  frc::MechanismLigament2d *_shooterMechLowerConnector =
    _shooterMechRoot->Append<frc::MechanismLigament2d>("shooterLowerConnector", 0.05, -90_deg, 0);
  MechanismCircle2d _shooterMechBottomRoller{_shooterMechLowerConnector, "shooterBottomRoller", 0.025, 0_deg};
};