// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubIntake.h"

#include <units/current.h>
#include <utilities/Logger.h>

SubIntake::SubIntake() {
  _intakeMotorConfig.SmartCurrentLimit(60);
  _intakeMotor.OverwriteConfig(_intakeMotorConfig);
  Logger::Log("Intake/Intake Motor", &_intakeMotor);
}

frc2::CommandPtr SubIntake::IntakeOn() {
  return StartEnd([this] { _intakeMotor.Set(1); }, [this] { _intakeMotor.Set(0); });
};

frc2::CommandPtr SubIntake::IntakeOff() {
  return RunOnce([this] { _intakeMotor.Set(0); });
};

void SubIntake::CurrentHighTimer() {
  _intakeHighCurrentTimer.Start();

  if (_intakeHighCurrentTimer.Get() > 3_s) {
    intakeCurrentAlert.Set(true);
  }
};

// This method will be called once per scheduler run
void SubIntake::Periodic() {
  units::ampere_t current = _intakeMotor.GetOutputCurrent() * 1_A;
  Logger::Log("Intake/Intake Motor Current", current);
  if (current > 20_A) {
    CurrentHighTimer();
  } else {
    intakeCurrentAlert.Set(false);
    _intakeHighCurrentTimer.Reset();
  }

  units::celsius_t temperature = _intakeMotor.GetTemperature();
  Logger::Log("Intake/Intake Motor Temperature", temperature);

  if (temperature > 60_degC) {
    highTempuratureAlert.Set(true);
  } else {
    highTempuratureAlert.Set(false);
  }
}

void SubIntake::SimulationPeriodic() {
  _sim.SetInputVoltage(_intakeMotor.CalcSimVoltage());
  _sim.Update(20_ms);
  _intakeMotor.IterateSim(_sim.GetAngularVelocity());
}
