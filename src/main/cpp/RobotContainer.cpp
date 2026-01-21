// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/Commands.h>
#include "subsystems/SubDrivebase.h"
#include "commands/DriveCommands.h"
#include "subsystems/SubIntake.h"
#include "subsystems/SubFeeder.h"
#include "subsystems/SubIndexer.h"
#include "commands/AutonCommands.h"
#include "subsystems/SubVision.h"
#include "subsystems/SubTurret.h"
#include "subsystems/SubHood.h"
#include "subsystems/SubShooter.h"
#include "commands/VisionCommands.h"
#include "commands/TurretCommands.h"

#include "utilities/PoseHandler.h"

RobotContainer::RobotContainer() {
  // SubDrivebase::GetInstance().SetDefaultCommand(cmd::TeleopDrive(_driverController));
  SubDrivebase::GetInstance();
  ConfigureBindings();
  SubVision::GetInstance().SetDefaultCommand(cmd::AddVisionMeasurement());

  _autoManager.AddDefaultAuton(
    "default",  
    AutonHelper::MakeCommandPtrAuto(cmd::DefaultAuton())
  );

  frc::SmartDashboard::PutData("CHOSEN AUTON", &_autoManager.GetAutonChooser());

  SubTurret::GetInstance();
  SubHood::GetInstance();
  SubShooter::GetInstance();
}

void RobotContainer::ConfigureBindings() {
  _driverController.A().OnTrue(SubShooter::GetInstance().SetShooterTarget(1_tps));
  _driverController.Y().OnTrue(SubShooter::GetInstance().SetShooterTarget(3_tps));
  _driverController.B().OnTrue(SubShooter::GetInstance().SetShooterTarget(0_tps));
  _driverController.X().OnTrue(SubShooter::GetInstance().StopShooter());
  
  _driverController.POVUp().OnTrue(SubFeeder::GetInstance().FeederOn());
  _driverController.POVDown().OnTrue(SubFeeder::GetInstance().FeederOff());
}

std::shared_ptr<frc2::CommandPtr> RobotContainer::GetAutonomousCommand() {
  AutonHelper::AutonPtr chosen = _autoManager.GetChosenAuton();
  return chosen; 
}
