// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubIntake.h"

#include <units/current.h>
#include <utilities/Logger.h>

SubIntake::SubIntake() {
  _intakeMotorConfig.SmartCurrentLimit(60);
  _intakeMotor.OverwriteConfig(_intakeMotorConfig);
  _deployMotorConfig.SmartCurrentLimit(60); 
  _deployMotorConfig.softLimit.ForwardSoftLimit(DEPLOY_MAX_ANGLE.value());
  _deployMotorConfig.softLimit.ReverseSoftLimit(DEPLOY_MIN_ANGLE.value());
  _deployMotorConfig.encoder.PositionConversionFactor(1.0 / DEPLOY_GEARING);
  _deployMotorConfig.encoder.VelocityConversionFactor(1.0 / DEPLOY_GEARING);
  _deployMotorConfig.closedLoop.P(DEPLOY_P);
  _deployMotor.OverwriteConfig(_deployMotorConfig);
  Logger::Log("Intake/Intake Motor", &_intakeMotor);
}

frc2::CommandPtr SubIntake::IntakeOn() {
  return StartEnd([this] { _intakeMotor.Set(1.0); }, [this] { _intakeMotor.Set(0); });
}

frc2::CommandPtr SubIntake::IntakeOff() {
  return RunOnce([this] { _intakeMotor.Set(0); });
};

frc2::CommandPtr SubIntake::DeployIntake() {
  return StartEnd([this] { _deployMotor.SetPositionTarget(90_deg); },
    [this] { _deployMotor.SetPositionTarget(0_deg); });
};

frc2::CommandPtr SubIntake::RetractIntake() {
  return RunOnce([this] { _deployMotor.SetPositionTarget(0_deg); });
};

void SubIntake::EnableSoftLimit(bool enabled) {
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

frc2::CommandPtr SubIntake::ZeroDeploy() {
  return RunOnce([this] { _deployMotor.SetPosition(0_deg); });
};

frc2::CommandPtr SubIntake::DeployAutoZero() {
  return RunOnce([this] {
    EnableSoftLimit(false);
    _deployMotor.SetVoltage(-1_V);
    _currentlyZeroing = true;
    _hasZeroed = false;
  })
    .AndThen(frc2::cmd::WaitUntil(
      [this] { return abs(_deployMotor.GetOutputCurrent()) * 1_A > zeroingCurrentLimit; }))
    .AndThen(ZeroDeploy())
    .AndThen([this] {
      _deployMotor.StopMotor();
      _hasZeroed = true;
    })
    .FinallyDo([this] {
      _currentlyZeroing = false;
      EnableSoftLimit(true);
    });
};

void SubIntake::IntakeCurrentHighTimer() {
  _intakeHighCurrentTimer.Start();

  if (_intakeHighCurrentTimer.Get() > 3_s) {
    _intakeCurrentAlert.Set(true);
  }
};

void SubIntake::DeployCurrentHighTimer() {
  _deployHighCurrentTimer.Start();

  if (_deployHighCurrentTimer.Get() > 3_s) {
    deployCurrentAlert.Set(true);
  }
};

// This method will be called once per scheduler run
void SubIntake::Periodic() {
  units::ampere_t intakeCurrent = _intakeMotor.GetStatorCurrent();
  units::ampere_t deployCurrent = _deployMotor.GetStatorCurrent();
  Logger::Log("Intake/Intake Motor Current", intakeCurrent);
  Logger::Log("Intake/Deploy Motor Current", deployCurrent);

  if (intakeCurrent > 20_A) {
    IntakeCurrentHighTimer();
  } else {
    _intakeCurrentAlert.Set(false);
    _intakeHighCurrentTimer.Reset();
  }
  if (deployCurrent > 20_A) {
    DeployCurrentHighTimer();
  } else {
    deployCurrentAlert.Set(false);
    _deployHighCurrentTimer.Reset();
  }

  units::celsius_t intakeTemperature = _intakeMotor.GetTemperature();
  units::celsius_t deployTemperature = _deployMotor.GetTemperature();
  Logger::Log("Intake/Intake Motor Temperature", intakeTemperature);
  Logger::Log("Intake/Deploy Motor Temperature", deployTemperature);

  if (intakeTemperature > 60_degC) {
    intakeHighTemperatureAlert.Set(true);
  } else {
    intakeHighTemperatureAlert.Set(false);
  }
  if (deployTemperature > 60_degC) {
    deployHighTemperatureAlert.Set(true);
  } else {
    deployHighTemperatureAlert.Set(false);
  }
}

void SubIntake::SimulationPeriodic() {
  _sim.SetInputVoltage(_intakeMotor.CalcSimVoltage());
  _sim.Update(20_ms);
  _intakeMotor.IterateSim(_sim.GetAngularVelocity());
}
