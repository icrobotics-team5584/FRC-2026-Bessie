// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "utilities/AlertController.h"
#include "utilities/ICSparkFlex.h"
#include "utilities/MechanismCircle2d.h"

#include <frc/Alert.h>
#include <frc/Timer.h>
#include <frc/simulation/SingleJointedArmSim.h>
#include <frc/system/plant/DCMotor.h>
#include <frc/system/plant/LinearSystemId.h>
#include <frc2/command/SubsystemBase.h>

#include "Constants.h"
#include "frc2/command/Commands.h"
#include "rev/config/SparkFlexConfig.h"
#include "rev/config/SparkFlexConfigAccessor.h"
class SubDeploy : public frc2::SubsystemBase {
 public:
  static SubDeploy& GetInstance() {
    static SubDeploy instance;
    return instance;
  }
  SubDeploy();

  frc2::CommandPtr DeployIntake();
  frc2::CommandPtr ToggleDeploy();

  void EnableSoftLimit(bool enabled);
  frc2::CommandPtr ZeroDeploy();
  frc2::CommandPtr DeployAutoZero();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  void SimulationPeriodic() override;

 private:
  ICSparkFlex _deployMotor{canid::DEPLOY};
  rev::spark::SparkFlexConfig _deployMotorConfig;

  frc::Alert _deployHighTemperatureAlert{
    "Deploy Motor High Temperature!", frc::Alert::AlertType::kWarning};
  frc::Alert _deployCurrentAlert{"Deploy Motor Overcurrent!", frc::Alert::AlertType::kWarning};

  frc::Timer _deployHighCurrentTimer;

  motorAlertConfig DeployAlertConfig{_deployHighTemperatureAlert, _deployCurrentAlert, _deployHighCurrentTimer, 60_degC, 20_A};


  bool _hasZeroed = false;
  bool _currentlyZeroing = false;

  static constexpr units::ampere_t ZEROINGCURRENTLIMIT = 5_A;

  // Simulation components
  static constexpr double DEPLOY_P = 0.2;
  static constexpr double DEPLOY_GEARING = 2.0;
  static constexpr units::degree_t DEPLOY_MAX_ANGLE = 90_deg;
  static constexpr units::degree_t DEPLOY_MIN_ANGLE = 0_deg;
  static constexpr units::meter_t DEPLOY_ARM_LENGTH = 0.1_m;
  static constexpr units::degree_t DEPLOY_START_ANGLE = 0_deg;
  static constexpr units::kilogram_square_meter_t DEPLOY_MOI = 0.0000005_kg_sq_m;
  static constexpr frc::DCMotor DEPLOY_MOTOR_MODEL = frc::DCMotor::NeoVortex();
  frc::LinearSystem<2, 1, 2> _deployFlywheelSystem =
    frc::LinearSystemId::SingleJointedArmSystem(DEPLOY_MOTOR_MODEL, DEPLOY_MOI, DEPLOY_GEARING);
  frc::sim::SingleJointedArmSim _deploySim{_deployFlywheelSystem, DEPLOY_MOTOR_MODEL,
    DEPLOY_GEARING, DEPLOY_ARM_LENGTH, DEPLOY_MIN_ANGLE, DEPLOY_MAX_ANGLE, false,
    DEPLOY_START_ANGLE};
};
