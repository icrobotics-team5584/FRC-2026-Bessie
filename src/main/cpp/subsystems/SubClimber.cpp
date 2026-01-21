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
  if(_hasZeroed == false && _resetting == false) {
    _climberMotor.Set(0);
  }
  Logger::Log("Climber/Has Zeroed", _hasZeroed);
  Logger::Log("Climber/resettiing", _resetting);
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

/* Commands */
frc2::CommandPtr SubClimber::WaitUntilReset() {
  return frc2::cmd::RunOnce([this]{ _hasZeroed = false; }).AndThen(frc2::cmd::Run([this] {
    if(GetMotorCurrent() > _ZEROING_CURRENT && GetRightMotorCurrent() > _ZEROING_CURRENT) {
      _hasZeroed = true;
    }
    if (frc::RobotBase::IsSimulation() == true) {
      _hasZeroed = true;
    }
  })).Until([this]{ return _hasZeroed; });
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
