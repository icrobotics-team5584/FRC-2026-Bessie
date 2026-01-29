// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <frc2/command/CommandScheduler.h>
#include "utilities/ShiftHandler.h"
#include "utilities/Logger.h"
#include "subsystems/SubTurret.h"
#include "subsystems/SubHood.h"
#include "utilities/PoseHandler.h"
#include "utilities/ShotPlanner.h"

#include <frc/geometry/Transform2d.h>

Robot::Robot() {
  // arrayPublisher = nt::NetworkTableInstance::GetDefault().GetStructArrayTopic<frc::Pose3d>(std::string_view{"ZeroedComponentPoses"}).Publish();
  arrayPublisher = nt::NetworkTableInstance::GetDefault().GetStructArrayTopic<frc::Pose3d>(std::string_view{"FinalComponentPoses"}).Publish();
}

void Robot::RobotPeriodic() {
  frc2::CommandScheduler::GetInstance().Run();
  _finalRobotComponentsArray[0] = frc::Pose3d(_finalRobotComponentsArray[0].Translation(), frc::Rotation3d{0_deg,0_deg, SubTurret::GetInstance().GetTurretAngle()});
  _finalRobotComponentsArray[1] = frc::Pose3d(_finalRobotComponentsArray[1].Translation(), frc::Rotation3d{0_deg,-SubHood::GetInstance().GetHoodAngle(), SubTurret::GetInstance().GetTurretAngle()});
  arrayPublisher.Set(_finalRobotComponentsArray);
  Logger::Log("RebuiltShift/Hub Active", ShiftHandler::IsActiveShift());
  Logger::Log("RebuiltShift/Won Auton Shift", ShiftHandler::GetShiftName(ShiftHandler::GetWinningShift()));
  Logger::Log("RebuiltShift/Current Shift", ShiftHandler::GetShiftName(ShiftHandler::GetCurrentShift()));
  Logger::Log("RebuiltShift/Seconds Left on Shift", ShiftHandler::GetTimeLeft());

  frc::Translation3d shotTarget =
    ShotPlanner::CalculateShotTarget(PoseHandler::GetInstance().GetPose());
  Logger::FieldDisplay::GetInstance().DisplayPose(
    "Shot Target", ShotPlanner::ConvertToPose2d(shotTarget));

  Logger::Log("Robot/RioBrownOut", frc::RobotController::IsBrownedOut());
  Logger::Log("Robot/RioInputVoltage", frc::RobotController::GetInputVoltage() * 1_V);
  Logger::Log("Robot/RioInputCurrent", frc::RobotController::GetInputCurrent() * 1_A);
  Logger::Log("Robot/BatteryVoltage", frc::RobotController::GetBatteryVoltage());
  Logger::Log("Robot/PDHInputVoltage", m_pdh.GetVoltage() * 1_V);
  Logger::Log("Robot/PDHTotalCurrent", m_pdh.GetTotalCurrent() * 1_A);
}
void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::DisabledExit() {}

void Robot::AutonomousInit() {
  m_autonomousCommand = m_container.GetAutonomousCommand();

  if (m_autonomousCommand) {
    /* m_autonomousCommand is a shared_ptr<frc2::CommandPtr>. Compiler gets mad
     * at a single .get() but .Schedule has a override for a raw pointer to the
     * Command. Ergo, the second ->get() gets the raw pointer.
     */
    frc2::CommandScheduler::GetInstance().Schedule(m_autonomousCommand.get()->get());
  }
}

void Robot::AutonomousPeriodic() {}

void Robot::AutonomousExit() {}

void Robot::TeleopInit() {
  if (m_autonomousCommand) {
    m_autonomousCommand->Cancel();
  }
}

void Robot::TeleopPeriodic() {}

void Robot::TeleopExit() {}

void Robot::TestInit() {
  frc2::CommandScheduler::GetInstance().CancelAll();
}

void Robot::TestPeriodic() {}

void Robot::TestExit() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
