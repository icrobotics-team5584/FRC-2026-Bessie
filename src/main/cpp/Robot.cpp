// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include "utilities/ShiftHandler.h"
#include "utilities/Logger.h"
#include "utilities/PoseHandler.h"
#include "utilities/ShotPlanner.h"

#include <frc2/command/CommandScheduler.h>
#include <frc/DataLogManager.h>

#include <frc/geometry/Transform2d.h>

Robot::Robot() {
  //USB logging
  frc::DataLogManager::Start();
  frc::SmartDashboard::PutData(&frc2::CommandScheduler::GetInstance());
  frc::DriverStation::StartDataLog(frc::DataLogManager::GetLog());
  
}

void Robot::RobotPeriodic() {
  frc2::CommandScheduler::GetInstance().Run();

  Logger::Log("RebuiltShift/Hub Active", ShiftHandler::GetInstance().IsActiveShift());
  Logger::Log("RebuiltShift/Won Auton Shift", ShiftHandler::GetInstance().GetShiftName(ShiftHandler::GetInstance().GetWinningShift()));
  Logger::Log("RebuiltShift/Current Shift", ShiftHandler::GetInstance().GetShiftName(ShiftHandler::GetInstance().GetCurrentShift()));
  Logger::Log("RebuiltShift/Seconds Left on Shift", ShiftHandler::GetInstance().GetTimeLeft());
  Logger::Log("RebuiltShift/Override Active", ShiftHandler::GetInstance().GetOverrideActive());
  Logger::Log("Shot Planner/Override Active", ShotPlanner::GetOverrideEnabled());

  ShotPlanner::ShotPlannerResults shotTarget =
    ShotPlanner::CalculateShotTarget(PoseHandler::GetInstance().GetPose());
  Logger::FieldDisplay::GetInstance().DisplayPose(
    "Shot Target", ShotPlanner::ConvertToPose2d(shotTarget.targetPosition));

  Logger::Log("Robot/RioBrownOut", frc::RobotController::IsBrownedOut());
  Logger::Log("Robot/RioInputVoltage", frc::RobotController::GetInputVoltage() * 1_V);
  Logger::Log("Robot/RioInputCurrent", frc::RobotController::GetInputCurrent() * 1_A);
  Logger::Log("Robot/BatteryVoltage", frc::RobotController::GetBatteryVoltage());
  Logger::Log("Robot/PDHInputVoltage", m_pdh.GetVoltage() * 1_V);
  Logger::Log("Robot/PDHTotalCurrent", m_pdh.GetTotalCurrent() * 1_A);

   Logger::Log("Shot Planner/Should Shoot", shotTarget.shouldShoot);
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
