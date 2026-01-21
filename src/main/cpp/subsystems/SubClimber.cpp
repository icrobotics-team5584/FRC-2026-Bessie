// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubClimber.h"
#include "utilities/Logger.h"

SubClimber::SubClimber() {
  _climberMotorConfig.encoder.PositionConversionFactor(1 / _GEAR_RATIO);
  _climberMotorConfig.encoder.VelocityConversionFactor(1 / _GEAR_RATIO);
  _climberMotorConfig.SmartCurrentLimit(60);                             /* Amps */
  _climberMotorConfig.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake);
  _climberMotorConfig.closedLoop.Pid(_P, _I, _D, rev::spark::ClosedLoopSlot::kSlot0);

  _climberMotor.OverwriteConfig(_climberMotorConfig);

  Logger::Log("Climber/Climber Motor", &_climberMotor);
  Logger::Log("Climber/PID Tolerance", _TOLERANCE);
}

// This method will be called once per scheduler run
void SubClimber::Periodic() {
  if(_hasZeroed == false && _zeroing == false) {
    _climberMotor.Set(0);
  }
  Logger::Log("Climber/Has Zeroed", _hasZeroed);
  Logger::Log("Climber/resettiing", _zeroing);
}

void SubClimber::SimulationPeriodic() {}

/* Instaneous */
void SubClimber::SetBrakeMode(bool isbrake) {
  rev::spark::SparkBaseConfig neutralModeConfig;
  if (isbrake) {
    neutralModeConfig.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake);
  } else {
    neutralModeConfig.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kCoast);
  }

  _climberMotor.AdjustConfigNoPersist(neutralModeConfig);
}

bool SubClimber::IsAtTarget() {
  units::turn_t currentPosition = _climberMotor.GetPosition();
  units::turn_t currentTarget = _climberMotor.GetPositionTarget();

  if (currentPosition < (currentTarget + _TOLERANCE) && currentPosition > (currentTarget - _TOLERANCE)) {
    return true;
  } else {
    return false;
  }
}

units::ampere_t SubClimber::GetMotorCurrent() {
  return (units::ampere_t)_climberMotor.GetOutputCurrent();
}

frc2::CommandPtr SubClimber::StowClimber() {
  _climberMotor.SetPositionTarget(_STOW_TURNS);
}

frc2::CommandPtr SubClimber::ReadyClimber() {
  _climberMotor.SetPositionTarget(_READY_TURNS);
}

frc2::CommandPtr SubClimber::ClimbL1() {
  _climberMotor.SetPositionTarget(_L1_TURNS);
}

frc2::CommandPtr SubClimber::RunCurrentZeroingSequence() {
  return RunOnce([this] {
    _hasZeroed = false;
    _zeroing = true;
    _climberMotor.SetVoltage(-1_V);
  }).AndThen(frc2::cmd::WaitUntil([this] {
    return (GetMotorCurrent() > _ZEROING_CURRENT) || (frc::RobotBase::IsSimulation() == true);
  })).FinallyDo([this] {
      _climberMotor.StopMotor();
      _climberMotor.SetPosition(0_deg);
      _zeroing = false;
      _hasZeroed = true;
  });
}