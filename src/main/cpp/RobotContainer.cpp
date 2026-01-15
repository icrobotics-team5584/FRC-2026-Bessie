// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/Commands.h>
#include "subsystems/SubDrivebase.h"
#include "commands/DriveCommands.h"
#include "subsystems/SubIntake.h"
#include "commands/AutonCommands.h"
#include "subsystems/SubStorage.h"
#include <commands/FuelCommands.h>
#include "Subsystems/SubVision.h"

RobotContainer::RobotContainer() {
  SubDrivebase::GetInstance().SetDefaultCommand(cmd::TeleopDrive(_driverController));
  ConfigureBindings();
  SubVision::GetInstance();

  _autoManager.AddDefaultAuton(
    "default",
    AutonHelper::MakeCommandPtrAuto(cmd::DefaultAuton())
  );

  frc::SmartDashboard::PutData("CHOSEN AUTON:", &_autoManager.GetAutonChooser());
}

void RobotContainer::ConfigureBindings() {
  

  _driverController.X().WhileTrue(SubDrivebase::GetInstance().CharacteriseWheels());
<<<<<<< HEAD
  _driverController.Y().WhileTrue(cmd::IntakeSequence());
=======
  _driverController.Y().OnTrue(SubDrivebase::GetInstance().ResetGyroCmd());
  _driverController.B().OnTrue(SubDrivebase::GetInstance().SyncSensor());
>>>>>>> main
}

std::shared_ptr<frc2::CommandPtr> RobotContainer::GetAutonomousCommand() {
  AutonHelper::AutonPtr chosen = _autoManager.GetChosenAuton();
  return chosen; 
}
