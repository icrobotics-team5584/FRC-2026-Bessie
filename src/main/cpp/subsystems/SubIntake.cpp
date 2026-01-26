// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubIntake.h"

#include <units/current.h>
#include <utilities/Logger.h>
#include "utilities/RobotVisualisation.h"

SubIntake::SubIntake() {
  _intakeMotorConfig.SmartCurrentLimit(60);
  _intakeMotor.OverwriteConfig(_intakeMotorConfig);

  Logger::Log("Intake/Intake Motor", &_intakeMotor);
}

frc2::CommandPtr SubIntake::IntakeOn() {
  return StartEnd([this] { _intakeMotor.Set(1.0); }, [this] { _intakeMotor.Set(0); });
}

frc2::CommandPtr SubIntake::IntakeOff() {
  return RunOnce([this] { _intakeMotor.Set(0); });
}

void SubIntake::IntakeCurrentHighTimer() {
  _intakeHighCurrentTimer.Start();

  if (_intakeHighCurrentTimer.Get() > 3_s) {
    _intakeCurrentAlert.Set(true);
  }
}

// This method will be called once per scheduler run
void SubIntake::Periodic() {
  units::ampere_t intakeCurrent = _intakeMotor.GetStatorCurrent();

  Logger::Log("Intake/Intake Motor Current", intakeCurrent);

  if (intakeCurrent > 20_A) {
    IntakeCurrentHighTimer();
  } else {
    _intakeCurrentAlert.Set(false);
    _intakeHighCurrentTimer.Reset();
  }

  units::celsius_t intakeTemperature = _intakeMotor.GetTemperature();

  Logger::Log("Intake/Intake Motor Temperature", intakeTemperature);

  if (intakeTemperature > 60_degC) {
    _intakeHighTemperatureAlert.Set(true);
  } else {
    _intakeHighTemperatureAlert.Set(false);
  }
  RobotVisualisation::GetInstance()._intakeWheel.SetAngle(_intakeMotor.GetPosition());
}

void SubIntake::SimulationPeriodic() {
  _sim.SetInputVoltage(_intakeMotor.CalcSimVoltage());
  _sim.Update(20_ms);
  _intakeMotor.IterateSim(_sim.GetAngularVelocity());
}

