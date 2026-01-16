// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/Commands.h>
#include "subsystems/SubDrivebase.h"
#include "commands/DriveCommands.h"
#include "subsystems/SubIntake.h"
#include "commands/AutonCommands.h"
#include "Subsystems/SubVision.h"
#include "subsystems/SubTurret.h"
#include "subsystems/SubHood.h"
#include "subsystems/SubShooter.h"
#include "subsystems/SubTurret.h"
#include "commands/DriveCommands.h"
#include "commands/ShootCommands.h"

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
  _driverController.Y().OnTrue(SubDrivebase::GetInstance().ResetGyroCmd());
  _driverController.B().OnTrue(SubDrivebase::GetInstance().SyncSensor());

  _driverController.A().OnTrue(SubTurret::GetInstance().ZeroTurretCmd());
  _driverController.RightBumper().OnTrue(SubTurret::GetInstance().SetTurretTargetAngle(45_deg));
  _driverController.LeftBumper().OnTrue(SubTurret::GetInstance().SetTurretTargetAngle(-45_deg));

  _driverController.RightTrigger().WhileTrue(cmd::AimAndShoot({0_m, 0_m, 0_m}));
}

std::shared_ptr<frc2::CommandPtr> RobotContainer::GetAutonomousCommand() {
  AutonHelper::AutonPtr chosen = _autoManager.GetChosenAuton();
  return chosen; 
}