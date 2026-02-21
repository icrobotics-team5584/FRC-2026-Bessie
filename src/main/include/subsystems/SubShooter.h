// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "utilities/AlertController.h"
#include "utilities/MechanismCircle2d.h"

#include <frc/Alert.h>
#include <frc/simulation/FlywheelSim.h>
#include <frc/smartdashboard/Mechanism2d.h>
#include <frc/smartdashboard/MechanismLigament2d.h>
#include <frc/system/plant/DCMotor.h>
#include <frc/system/plant/LinearSystemId.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>
#include <frc2/command/SubsystemBase.h>

#include <ctre/phoenix6/TalonFX.hpp>
#include <wpi/interpolating_map.h>

#include "Constants.h"

class SubShooter : public frc2::SubsystemBase {
 public:
  SubShooter();
  static SubShooter& GetInstance() {
    static SubShooter inst;
    return inst;
  }

  void SimulationPeriodic();

  frc2::CommandPtr SetShooterTarget(std::function<units::turns_per_second_t()> speed);
  frc2::CommandPtr StopShooter();
  frc2::CommandPtr SpinWithDistance(std::function<units::meter_t()> distance, std::function<bool()> isPassing);
  frc2::CommandPtr SpinShooterSlowly();
  
  bool IsAtSpeed();

  units::second_t GetTimeOfFLightWithDistance(units::meter_t distance);

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

  frc::Alert _shooter1highTemperatureAlert{
    "Shooter Motor 1 High Temperature!", frc::Alert::AlertType::kWarning};
  frc::Alert _shooter1CurrentAlert{"Shooter Motor 1 Overcurrent!", frc::Alert::AlertType::kWarning};

  frc::Alert _shooter1RecordedTemperatureAlert{
    "Shooter Motor 1 max Temperature was reached !", frc::Alert::AlertType::kWarning};

  frc::Alert _shooter1RecordedCurrentAlert{
    "Shooter Motor 1 max current was reached !", frc::Alert::AlertType::kWarning};

  AlertController::MotorAlertConfig _shooter1AlertConfig{_shooter1highTemperatureAlert,
    _shooter1CurrentAlert, _shooter1RecordedTemperatureAlert, _shooter1RecordedCurrentAlert,
    60_degC, 20_A};

  frc::Alert _shooter2highTemperatureAlert{
    "Shooter Motor 2 High Temperature!", frc::Alert::AlertType::kWarning};
  frc::Alert _shooter2CurrentAlert{"Shooter Motor 2 Overcurrent!", frc::Alert::AlertType::kWarning};

  frc::Alert _shooter2RecordedTemperatureAlert{
    "Shooter Motor 2 max Temperature was reached !", frc::Alert::AlertType::kWarning};

  frc::Alert _shooter2RecordedCurrentAlert{
    "Shooter Motor 2 max current was reached !", frc::Alert::AlertType::kWarning};

  AlertController::MotorAlertConfig _shooter2AlertConfig{_shooter2highTemperatureAlert,
    _shooter2CurrentAlert, _shooter2RecordedTemperatureAlert, _shooter2RecordedCurrentAlert,
    60_degC, 20_A};

  wpi::interpolating_map<units::meter_t, units::turns_per_second_t> _flyWheelSpeedTableScoring;
  wpi::interpolating_map<units::meter_t, units::turns_per_second_t> _flyWheelSpeedTablePassing;
  wpi::interpolating_map<units::meter_t, units::second_t> _timeOfFlightTable;

  // Sim
  frc::LinearSystem<1, 1, 1> _leftFlywheelSystem =
    frc::LinearSystemId::FlywheelSystem(MOTOR_MODEL, MOI, GEAR_RATIO);
  frc::sim::FlywheelSim _leftFlywheelSim{_leftFlywheelSystem, MOTOR_MODEL};

  frc::LinearSystem<1, 1, 1> _rightFlywheelSystem =
    frc::LinearSystemId::FlywheelSystem(MOTOR_MODEL, MOI, GEAR_RATIO);
  frc::sim::FlywheelSim _rightFlywheelSim{_rightFlywheelSystem, MOTOR_MODEL};

  // mechanism2d
  frc::Mechanism2d _shooterMech{0.25, 0.25};
  frc::MechanismRoot2d* _shooterMechRoot = _shooterMech.GetRoot("shooterRoot", 0.125, 0.125);
  frc::MechanismLigament2d* _shooterMechUpperConnector =
    _shooterMechRoot->Append<frc::MechanismLigament2d>("shooterUpperConnector", 0.05, 90_deg, 0);
  MechanismCircle2d _shooterMechTopRoller{
    _shooterMechUpperConnector, "shooterTopRoller", 0.025, 0_deg};
  frc::MechanismLigament2d* _shooterMechLowerConnector =
    _shooterMechRoot->Append<frc::MechanismLigament2d>("shooterLowerConnector", 0.05, -90_deg, 0);
  MechanismCircle2d _shooterMechBottomRoller{
    _shooterMechLowerConnector, "shooterBottomRoller", 0.025, 0_deg};
};
