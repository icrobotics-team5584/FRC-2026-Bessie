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
  SubTurret::GetInstance();
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
  _driverController.X().OnTrue(SubHood::GetInstance().SetHoodPosition(0_deg));
  _driverController.Y().OnTrue(SubHood::GetInstance().SetHoodPosition(20_deg));
  _driverController.A().OnTrue(SubShooter::GetInstance().SetShooterTarget(100_tps));
  _driverController.B().OnTrue(SubShooter::GetInstance().StopShooter());
  _driverController.RightBumper().WhileTrue(cmd::AimAndShoot({0_m, 0_m, 0_m})); // Change to hub pose later
}

std::shared_ptr<frc2::CommandPtr> RobotContainer::GetAutonomousCommand() {
  AutonHelper::AutonPtr chosen = _autoManager.GetChosenAuton();
  return chosen; 
}