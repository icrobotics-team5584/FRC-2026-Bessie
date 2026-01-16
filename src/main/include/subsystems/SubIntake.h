// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "utilities/ICSparkFlex.h"

#include <frc/Alert.h>
#include <frc/Timer.h>
#include <frc/simulation/DCMotorSim.h>
#include <frc/simulation/FlywheelSim.h>
#include <frc/system/plant/DCMotor.h>
#include <frc/system/plant/LinearSystemId.h>
#include <frc2/command/SubsystemBase.h>

#include "Constants.h"
#include "frc2/command/Commands.h"
#include "frc2/command/SubsystemBase.h"
#include "rev/config/SparkFlexConfig.h"
#include "rev/config/SparkFlexConfigAccessor.h"

class SubIntake : public frc2::SubsystemBase {
 public:
  static SubIntake& GetInstance() {
    static SubIntake instance;
    return instance;
  }
  SubIntake();

  frc2::CommandPtr IntakeOn();
  frc2::CommandPtr IntakeOff();

  frc2::CommandPtr DeployIntake();
  frc2::CommandPtr RetractIntake();

  void IntakeCurrentHighTimer();
  void DeployCurrentHighTimer();

  frc::Alert intakeCurrentAlert{"Intake Motor Overcurrent!", frc::Alert::AlertType::kWarning};
  frc::Alert deployCurrentAlert{"Deploy Motor Overcurrent!", frc::Alert::AlertType::kWarning};
  frc::Alert intakeHighTempuratureAlert{
    "Intake Motor High Temperature!", frc::Alert::AlertType::kWarning};
  frc::Alert deployHighTemperatureAlert{
    "Deploy Motor High Temperature!", frc::Alert::AlertType::kWarning};

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  void SimulationPeriodic() override;

 private:
  ICSparkFlex _intakeMotor{canid::INTAKE};
  ICSparkFlex _deployMotor{canid::INTAKE_DEPLOY};

  rev::spark::SparkFlexConfig _intakeMotorConfig;
  rev::spark::SparkFlexConfig _deployMotorConfig;

  frc::Timer _intakeHighCurrentTimer;
  frc::Timer _deployHighCurrentTimer;
 
  // Simulation components
  static constexpr double GEARING = 1.0;
  static constexpr double DEPLOY_P = 0.2;
  static constexpr units::kilogram_square_meter_t MOI = 0.0000001_kg_sq_m;
  static constexpr frc::DCMotor MOTOR_MODEL = frc::DCMotor::NeoVortex();
  frc::LinearSystem<1, 1, 1> _flywheelSystem =
    frc::LinearSystemId::FlywheelSystem(MOTOR_MODEL, MOI, GEARING);
  frc::sim::FlywheelSim _sim{_flywheelSystem, MOTOR_MODEL};

  static constexpr double DEPLOY_GEARING = 2.0;
  static constexpr units::kilogram_square_meter_t DEPLOY_MOI = 0.0000005_kg_sq_m;
  static constexpr frc::DCMotor DEPLOY_MOTOR_MODEL = frc::DCMotor::NeoVortex();
  frc::LinearSystem<2, 1, 2> _deployFlywheelSystem =
    frc::LinearSystemId::DCMotorSystem(DEPLOY_MOTOR_MODEL, DEPLOY_MOI, DEPLOY_GEARING);
  frc::sim::DCMotorSim _deploySim{_deployFlywheelSystem, DEPLOY_MOTOR_MODEL};
};