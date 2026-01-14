// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/Commands.h>
#include "subsystems/SubDrivebase.h"
#include "commands/DriveCommands.h"
#include "commands/AutonCommands.h"

RobotContainer::RobotContainer() {
  SubDrivebase::GetInstance().SetDefaultCommand(cmd::TeleopDrive(_driverController));
  ConfigureBindings();
  
  _autoManager.AddDefaultAuton("default", AutonHelper::MakeCommandPtrAuto(cmd::DefaultAuton()));
  _autoManager.AddAuton("forward", AutonHelper::MakePathPlannerAuto(pathplanner::PathPlannerAuto("Forward")));

  frc::SmartDashboard::PutData("Chosen Autonomous Routine", &_autoManager.GetAutonChooser());
}

void RobotContainer::ConfigureBindings() {
  _driverController.X().WhileTrue(SubDrivebase::GetInstance().CharacteriseWheels());
}

std::shared_ptr<frc2::CommandPtr> RobotContainer::GetAutonomousCommand() {
  AutonHelper::AutonPtr chosen = _autoManager.GetChosenAuton();
  return chosen; 
}
