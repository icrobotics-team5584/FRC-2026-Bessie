// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubDeploy.h"

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
  return RunOnce([this] { _deployMotor.SetPositionTarget(90_deg); });
}

frc2::CommandPtr SubDeploy::ToggleDeploy() {
  return RunOnce([this] {
    if (_deployMotor.GetPosition() > 45_deg) {
      _deployMotor.SetPositionTarget(0_deg);
    } else {
      _deployMotor.SetPositionTarget(90_deg);
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

void SubDeploy::DeployCurrentHighTimer() {
  _deployHighCurrentTimer.Start();

  if (_deployHighCurrentTimer.Get() > 3_s) {
    _deployCurrentAlert.Set(true);
  }
}

// This method will be called once per scheduler run
void SubDeploy::Periodic() {
  units::ampere_t deployCurrent = _deployMotor.GetStatorCurrent();
  Logger::Log("Deploy/Deploy Motor Current", deployCurrent);

  if (deployCurrent > 20_A) {
    DeployCurrentHighTimer();
  } else {
    _deployCurrentAlert.Set(false);
    _deployHighCurrentTimer.Reset();
  }

  units::celsius_t deployTemperature = _deployMotor.GetTemperature();
  Logger::Log("Deploy/Deploy Motor Temperature", deployTemperature);

  if (deployTemperature > 60_degC) {
    _deployHighTemperatureAlert.Set(true);
  } else {
    _deployHighTemperatureAlert.Set(false);
  }
}

void SubDeploy::SimulationPeriodic() {
  _deploySim.SetInputVoltage(_deployMotor.CalcSimVoltage());
  _deploySim.Update(20_ms);
  _deployMotor.IterateSim(_deploySim.GetVelocity(), _deploySim.GetAngle());
}
