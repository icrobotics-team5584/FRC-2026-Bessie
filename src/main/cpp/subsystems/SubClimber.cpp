// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubClimber.h"

SubClimber::SubClimber() {
  _climbMotorConfig.encoder.PositionConversionFactor(1 / GEAR_RATIO);
  _climbMotorConfig.encoder.VelocityConversionFactor(1 / GEAR_RATIO);
  _climbMotorConfig.SmartCurrentLimit(60);                             /* Amps */
  _climbMotorConfig.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake);
  _climbMotorConfig.closedLoop.Pid(P, I, D, rev::spark::ClosedLoopSlot::kSlot0);

  _leftClimbMotor.OverwriteConfig(_climbMotorConfig);
  _rightClimbMotor.OverwriteConfig(_climbMotorConfig);

  frc::SmartDashboard::PutData("Climber/topClimbMotor", &_leftClimbMotor);
  frc::SmartDashboard::PutData("Climber/bottomClimbMotor", &_rightClimbMotor);
  frc::SmartDashboard::PutNumber("Climber/target tolerance", TOLERANCE.value());
  frc::SmartDashboard::PutBoolean("Climber/has reset", _hasReset);
  frc::SmartDashboard::PutBoolean("Climber/resettiing", _resetting);
}

// This method will be called once per scheduler run
void SubClimber::Periodic() {
    if(_hasReset == false && _resetting == false) {
        _leftClimbMotor.Set(0);
        _rightClimbMotor.Set(0);
    }
    frc::SmartDashboard::PutBoolean("Climber/has reset", _hasReset);
    frc::SmartDashboard::PutBoolean("Climber/resettiing", _resetting);
}

void SubClimber::SimulationPeriodic() {}

/* Instaneous */
void SubClimber::SetBrakeMode(bool isbrake) {
  rev::spark::SparkBaseConfig mode;
  if (isbrake) {
    mode.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake);
  } else {
    mode.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake);
  }
  _leftClimbMotor.AdjustConfigNoPersist(mode);
  _rightClimbMotor.AdjustConfigNoPersist(mode);
}

bool SubClimber::IsAtTarget() {
  bool rightattarget = false, leftattarget = false;
  units::turn_t leftrot = _leftClimbMotor.GetPosition();
  units::turn_t rightrot = _leftClimbMotor.GetPosition();
  units::turn_t lefttarget = _leftClimbMotor.GetPositionTarget();
  units::turn_t righttarget = _leftClimbMotor.GetPositionTarget();

  if (rightrot < righttarget + TOLERANCE && rightrot > righttarget - TOLERANCE) {
    rightattarget = true;
  }

  if (leftrot < lefttarget + TOLERANCE && leftrot > lefttarget - TOLERANCE) {
    leftattarget = true;
  }

  return rightattarget && leftattarget;
}

units::ampere_t  SubClimber::GetLeftMotorCurrent()
{
    return (units::ampere_t)_leftClimbMotor.GetOutputCurrent();
}

units::ampere_t  SubClimber::GetRightMotorCurrent()
{
    return (units::ampere_t)_rightClimbMotor.GetOutputCurrent();
}


/* Commands */
frc2::CommandPtr SubClimber::WaitUntilReset()
{
    return frc2::cmd::RunOnce([this]{ _hasReset = false; })
        .AndThen(frc2::cmd::Run([this]
        {
            if(GetLeftMotorCurrent() > ZEROING_CURRENT && GetRightMotorCurrent() > ZEROING_CURRENT) {
                _hasReset = true;
            }
            if (frc::RobotBase::IsSimulation() == true) {
                _hasReset = true;
            }
        }))
        .Until([this]{ return _hasReset; });
}

frc2::CommandPtr SubClimber::ReadyClimber() 
{
  _leftClimbMotor.SetPositionTarget(LEFT_READY_TURNS);
  _rightClimbMotor.SetPositionTarget(RIGHT_READY_TURNS);
}

frc2::CommandPtr SubClimber::ClimbL1() 
{
  _leftClimbMotor.SetPositionTarget(LEFT_L1_TURNS);
  _rightClimbMotor.SetPositionTarget(RIGHT_L1_TURNS);
}

// frc2::CommandPtr SubClimber::ClimbL2() 
// {
//   _leftClimbMotor.SetPositionTarget(LEFT_L2_TURNS);
//   _rightClimbMotor.SetPositionTarget(RIGHT_L2_TURNS);
// }

// frc2::CommandPtr SubClimber::ClimbL3() {
//   _leftClimbMotor.SetPositionTarget(LEFT_L3_TURNS);
//   _rightClimbMotor.SetPositionTarget(RIGHT_L3_TURNS);
// }
