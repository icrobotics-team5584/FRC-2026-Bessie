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

// This method will be called once per scheduler run
void SubIntake::Periodic() {}
