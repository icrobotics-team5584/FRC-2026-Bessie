// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubDeploy.h"

#include "utilities/RobotVisualisation.h"

#include <units/current.h>
#include <utilities/Logger.h>

SubDeploy::SubDeploy() {
  _deployMotorConfig.SmartCurrentLimit(60);
  _deployMotorConfig.softLimit.ForwardSoftLimit(DEPLOY_MAX_ANGLE.value());
  _deployMotorConfig.softLimit.ReverseSoftLimit(DEPLOY_MIN_ANGLE.value());
  _deployMotorConfig.encoder.PositionConversionFactor(1.0 / DEPLOY_GEARING);
  _deployMotorConfig.encoder.VelocityConversionFactor(1.0 / DEPLOY_GEARING);
  _deployMotorConfig.closedLoop.P(DEPLOY_P);
  _deployMotor.OverwriteConfig(_deployMotorConfig);

  Logger::Log("Deploy/DeployMotor", &_deployMotor);
}

frc2::CommandPtr SubDeploy::DeployIntake() {
  return RunOnce([this] { _deployMotor.SetPositionTarget(DEPLOY_MIN_ANGLE); });
}

frc2::CommandPtr SubDeploy::ToggleDeploy() {
  return RunOnce([this] {
    if (_deployMotor.GetPosition() > 45_deg) {
      _deployMotor.SetPositionTarget(DEPLOY_MIN_ANGLE);
    } else {
      _deployMotor.SetPositionTarget(DEPLOY_MAX_ANGLE);
    }
  });
}

frc2::CommandPtr SubDeploy::ZeroDeploy() {
  return RunOnce([this] { _deployMotor.SetPosition(0_deg); });
}

frc2::CommandPtr SubDeploy::DeployAutoZero() {
  return RunOnce([this] {
    EnableSoftLimit(false);
    _deployMotor.SetVoltage(-1_V);
    _currentlyZeroing = true;
    _hasZeroed = false;
  })
    .AndThen(frc2::cmd::WaitUntil(
      [this] { return abs(_deployMotor.GetOutputCurrent()) * 1_A > ZEROINGCURRENTLIMIT; }))
    .AndThen(ZeroDeploy())
    .AndThen([this] {
      _deployMotor.StopMotor();
      _hasZeroed = true;
    })
    .FinallyDo([this] {
      _currentlyZeroing = false;
      EnableSoftLimit(true);
    });
}

void SubDeploy::EnableSoftLimit(bool enabled) {
  if (!enabled) {
    _deployMotorConfig.softLimit.ForwardSoftLimitEnabled(false);
    _deployMotorConfig.softLimit.ReverseSoftLimitEnabled(false);
    _deployMotor.AdjustConfig(_deployMotorConfig);
  } else {
    _deployMotorConfig.softLimit.ForwardSoftLimitEnabled(true);
    _deployMotorConfig.softLimit.ReverseSoftLimitEnabled(true);
    _deployMotor.AdjustConfig(_deployMotorConfig);
  }
}

// This method will be called once per scheduler run
void SubDeploy::Periodic() {
  units::ampere_t deployCurrent = _deployMotor.GetStatorCurrent();

  units::celsius_t deployTemperature = _deployMotor.GetTemperature();

  AlertController::UpdateTemperatureAlert(DeployAlertConfig, deployTemperature);
  AlertController::UpdateCurrentAlert(DeployAlertConfig, deployCurrent);


  RobotVisualisation::GetInstance()._deployLigament->SetAngle(_deployMotor.GetPosition());
}

void SubDeploy::SimulationPeriodic() {
  _deploySim.SetInputVoltage(_deployMotor.CalcSimVoltage());
  _deploySim.Update(20_ms);
  _deployMotor.IterateSim(_deploySim.GetVelocity(), _deploySim.GetAngle());
}
