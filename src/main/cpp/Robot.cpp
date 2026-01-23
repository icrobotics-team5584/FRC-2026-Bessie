// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <frc2/command/CommandScheduler.h>
#include <utilities/ICSparkFlex.h>
#include "utilities/ShotPlanner.h"
#include "utilities/PoseHandler.h"
#include "utilities/Logger.h"
#include <frc/geometry/Translation3d.h>
#include <frc/geometry/Transform2d.h>
Robot::Robot() {}

void Robot::RobotPeriodic() {
  frc2::CommandScheduler::GetInstance().Run();
  frc::Translation3d shotTarget = ShotPlanner::CalculateShotTarget(PoseHandler::GetInstance().GetPose());
  Logger::FieldDisplay::GetInstance().DisplayPose("Shot Target",ShotPlanner::convertToPose2d(shotTarget));
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
