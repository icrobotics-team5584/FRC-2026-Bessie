// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubDeploy.h"

#include "utilities/RobotVisualisation.h"

#include <units/current.h>
#include <utilities/Logger.h>

SubDeploy::SubDeploy() {
  _deployMotorConfig.SmartCurrentLimit(60);
  _deployMotorConfig.softLimit.ForwardSoftLimit(RETRACTED_ANGLE.value());
  _deployMotorConfig.softLimit.ReverseSoftLimit(DEPLOYED_ANGLE.value());
  _deployMotorConfig.encoder.PositionConversionFactor(1.0 / DEPLOY_GEARING);
  _deployMotorConfig.encoder.VelocityConversionFactor(1.0 / DEPLOY_GEARING);
  _deployMotorConfig.closedLoop.P(DEPLOY_P);
  _deployMotor.OverwriteConfig(_deployMotorConfig);

  Logger::Log("Deploy/DeployMotor", &_deployMotor);
}

frc2::CommandPtr SubDeploy::DeployIntake() {
  return RunOnce([this] { _deployMotor.SetPositionTarget(DEPLOYED_ANGLE); }).OnlyIf([this] {return _hasZeroed; });
}

frc2::CommandPtr SubDeploy::RetractIntake() {
  return RunOnce([this] { _deployMotor.SetPositionTarget(RETRACTED_ANGLE); }).OnlyIf([this] {return _hasZeroed; });
}

frc2::CommandPtr SubDeploy::ToggleDeploy() {
  return RunOnce([this] {
    if (_deployMotor.GetPosition() > RETRACTED_ANGLE/2.0) {
      _deployMotor.SetPositionTarget(DEPLOYED_ANGLE);
    } else {
      _deployMotor.SetPositionTarget(RETRACTED_ANGLE);
    }
  });
}

frc2::CommandPtr SubDeploy::Zero() {
  return RunOnce([this] {
    EnableSoftLimit(false);
    _deployMotor.SetVoltage(-3_V);
    _currentlyZeroing = true;
    _hasZeroed = false;
  })
    .AndThen(frc2::cmd::WaitUntil(
      [this] { return abs(_deployMotor.GetOutputCurrent()) * 1_A > ZEROINGCURRENTLIMIT; }))
    .AndThen([this] {
      _deployMotor.SetPosition(-1_deg);  // pushes into the bumpers about 1 degree when zeroing
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

void SubDeploy::SetBrakeMode(bool brakeMode){
    rev::spark::SparkBaseConfig _neutralModeConfig;
    if (brakeMode == true) {
      _neutralModeConfig.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake);
      _deployMotor.AdjustConfigNoPersist(_neutralModeConfig);
    } else if (brakeMode == false) {
      _neutralModeConfig.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kCoast);
      _deployMotor.AdjustConfigNoPersist(_neutralModeConfig);
    }
}

// This method will be called once per scheduler run
void SubDeploy::Periodic() {
  auto loopStart = frc::GetTime();

  units::ampere_t deployCurrent = _deployMotor.GetStatorCurrent();

  units::celsius_t deployTemperature = _deployMotor.GetTemperature();

  AlertController::UpdateTemperatureAlert(DeployAlertConfig, deployTemperature);
  AlertController::UpdateCurrentAlert(DeployAlertConfig, deployCurrent);

  RobotVisualisation::GetInstance()._deployLigament->SetAngle(_deployMotor.GetPosition());

  Logger::Log("Deploy/Loop Time", (frc::GetTime() - loopStart));
  Logger::Log("Deploy/IsZeroing", _currentlyZeroing);
  Logger::Log("Deploy/HasZeroed", _hasZeroed);

  if(_hasZeroed == false && _currentlyZeroing == false) {
    frc2::CommandScheduler::GetInstance().Schedule(Idle());
  }
}

void SubDeploy::SimulationPeriodic() {
  _deploySim.SetInputVoltage(_deployMotor.CalcSimVoltage());
  _deploySim.Update(20_ms);
  _deployMotor.IterateSim(_deploySim.GetVelocity(), _deploySim.GetAngle());
}
