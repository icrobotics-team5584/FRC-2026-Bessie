// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubClimber.h"

#include "utilities/Logger.h"
#include "utilities/RobotVisualisation.h"

SubClimber::SubClimber() {
  _climberMotorConfig.encoder.PositionConversionFactor(1 / _GEAR_RATIO);
  _climberMotorConfig.encoder.VelocityConversionFactor(1 / _GEAR_RATIO);
  _climberMotorConfig.SmartCurrentLimit(60); /* Amps */
  _climberMotorConfig.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake);
  _climberMotorConfig.closedLoop.Pid(_P, _I, _D, rev::spark::ClosedLoopSlot::kSlot0);

  _climberMotor.OverwriteConfig(_climberMotorConfig);

  Logger::Log("Climber/Climber Motor", &_climberMotor);
  Logger::Log("Climber/PID Tolerance", _TOLERANCE);
}

// This method will be called once per scheduler run
void SubClimber::Periodic() {
  if (_hasZeroed == false && _zeroing == false) {
    _climberMotor.Set(0);
  }
  Logger::Log("Climber/Has Zeroed", _hasZeroed);
  Logger::Log("Climber/Zeroing", _zeroing);
  Logger::Log("Climber/Distance", GetElevatorHeight());

  RobotVisualisation::GetInstance()._climberMechExtension->SetLength(
    (GetElevatorHeight() - _ELEVATOR_MIN_HEIGHT).value());
}

void SubClimber::SimulationPeriodic() {
  units::volt_t simVoltage = _climberMotor.CalcSimVoltage();
  _climberSim.SetInputVoltage(simVoltage);
  _climberSim.Update(20_ms);

  units::meter_t simPosition = _climberSim.GetPosition();
  units::meters_per_second_t simVelocity = _climberSim.GetVelocity();

  units::turn_t simRotorPosition = (simPosition / _DRUM_CIRCUMFERENCE) * 1_tr;
  units::turns_per_second_t simRotorVelocity = (simVelocity / _DRUM_CIRCUMFERENCE) * 1_tr;
  _climberMotor.IterateSim(simRotorVelocity, simRotorPosition);

  Logger::Log("Climber/Sim/Voltage", simVoltage);
  Logger::Log("Climber/Sim/Position", simPosition);
  Logger::Log("Climber/Sim/Velocity", simVelocity);
  Logger::Log("Climber/Sim/Rotor Position", simRotorPosition);
  Logger::Log("Climber/Sim/Rotor Velocity", simRotorVelocity);
};

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
  units::turn_t posError = units::math::abs(_climberMotor.GetPosError());

  if (posError < _TOLERANCE) {
    return true;
  } else {
    return false;
  }
}

units::degree_t SubClimber::CalcMotorPosFromHeight(units::meter_t height) {
  return 360_deg * ((height - _ELEVATOR_MIN_HEIGHT) / _DRUM_CIRCUMFERENCE).value();
}

units::meter_t SubClimber::GetElevatorHeight() {
  units::turn_t motorPos = _climberMotor.GetPosition();
  return _ELEVATOR_MIN_HEIGHT + motorPos.value() * _DRUM_CIRCUMFERENCE;
}

units::ampere_t SubClimber::GetMotorCurrent() {
  return (units::ampere_t)_climberMotor.GetOutputCurrent();
}


frc2::CommandPtr SubClimber::ToggleClimb() {
  units::meter_t stow_height = std::clamp(_STOW_HEIGHT, _ELEVATOR_MIN_HEIGHT, _ELEVATOR_MAX_HEIGHT);
  units::meter_t l1_height = std::clamp(_L1_HEIGHT, _ELEVATOR_MIN_HEIGHT, _ELEVATOR_MAX_HEIGHT);

  auto onTrue = [this, l1_height] {
    _climberMotor.SetPositionTarget(CalcMotorPosFromHeight(l1_height)); 
  };

  auto onFalse = [this, stow_height] {
    _climberMotor.SetPositionTarget(CalcMotorPosFromHeight(stow_height)); 
  };

  return StartEnd(onTrue, onFalse)
    .OnlyIf([this] {return _hasZeroed; });
}

frc2::CommandPtr SubClimber::RunCurrentZeroingSequence() {
  return RunOnce([this] {
    _hasZeroed = false;
    _zeroing = true;
    _climberMotor.SetVoltage(-1_V);
  })
    .AndThen(frc2::cmd::WaitUntil([this] {
      return (GetMotorCurrent() > _ZEROING_CURRENT) || (frc::RobotBase::IsSimulation() == true);
    }))
    .AndThen([this] {
      /* Set zero seperately from FinallyDo so that it won't set the climber as
       * zeroed if the command was cancelled. */
      _climberMotor.SetPosition(0_deg);
      _hasZeroed = true;
    })
    .FinallyDo([this] {
      _climberMotor.StopMotor();
      _zeroing = false;
    });
}

frc2::CommandPtr SubClimber::ManualClimberUp() {
  return StartEnd([this] { _climberMotor.SetVoltage(4_V); },
    [this] {
      auto targRot = _climberMotor.GetPosition();
      _climberMotor.SetPositionTarget(targRot);
    });
}

frc2::CommandPtr SubClimber::ManualClimberDown() {
  return StartEnd([this] { _climberMotor.SetVoltage(-4_V); },
    [this] {
      auto targRot = _climberMotor.GetPosition();
      _climberMotor.SetPositionTarget(targRot);
    });
}