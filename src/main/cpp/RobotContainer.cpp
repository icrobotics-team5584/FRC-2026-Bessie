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
#include "Subsystems/SubVision.h"
#include "subsystems/SubTurret.h"
#include "subsystems/SubHood.h"
#include "subsystems/SubShooter.h"
#include "commands/VisionCommands.h"

#include "utilities/PoseHandler.h"

RobotContainer::RobotContainer() {
  SubDrivebase::GetInstance().SetDefaultCommand(cmd::TeleopDrive(_driverController));
  ConfigureBindings();
  SubVision::GetInstance().SetDefaultCommand(cmd::AddVisionMeasurement());

  _autoManager.AddDefaultAuton("default", AutonHelper::MakeCommandPtrAuto(cmd::DefaultAuton()));
  _autoManager.AddAuton("driveInASquare", AutonHelper::MakeCommandPtrAuto(cmd::DriveInASquare()));
  _autoManager.AddAuton("forward 250cm", AutonHelper::MakeCommandPtrAuto(cmd::Forward250cm()));
  _autoManager.AddAuton("forward 250cm while turning", AutonHelper::MakeCommandPtrAuto(cmd::Forward250cmWhileTurning()));
  _autoManager.AddAuton("NeutralScoreAndClimb_LeftBump", AutonHelper::MakeCommandPtrAuto(cmd::NeutralScoreAndClimb_LeftBump()));
  //_autoManager.AddAuton("NeutralScoreAndClimb_RightBump", AutonHelper::MakeCommandPtrAuto(cmd::NeutralScoreAndClimb_RightBump()));
  //_autoManager.AddAuton("Hoard_LeftBump", AutonHelper::MakeCommandPtrAuto(cmd::Hoard_LeftBump()));
  //_autoManager.AddAuton("Hoard_RightBump", AutonHelper::MakeCommandPtrAuto(cmd::Hoard_RightBump()));
  //_autoManager.AddAuton("OutpostDepotClimb", AutonHelper::MakeCommandPtrAuto(cmd::OutpostDepotClimb()));

  frc::SmartDashboard::PutData("CHOSEN AUTON", &_autoManager.GetAutonChooser());

  SubTurret::GetInstance();
  SubHood::GetInstance();
  SubShooter::GetInstance();
}

void RobotContainer::ConfigureBindings() {
  _driverController.X().WhileTrue(SubDrivebase::GetInstance().CharacteriseWheels());
  _driverController.Y().OnTrue(SubDrivebase::GetInstance().ResetGyroCmd());
  _driverController.B().OnTrue(SubDrivebase::GetInstance().SyncSensor());
  _driverController.A().OnTrue(frc2::cmd::RunOnce([]{
    SubDrivebase::GetInstance().SetPose(frc::Pose2d{3.3_m,5.4_m,0_deg});
  }));
  _driverController.B().OnTrue(SubDrivebase::GetInstance().SyncSensor());
  _driverController.LeftTrigger().OnTrue(frc2::cmd::RunOnce([]{
    SubDrivebase::GetInstance().SetPose(frc::Pose2d{7.4_m,5.4_m,0_deg});
  }));
}

std::shared_ptr<frc2::CommandPtr> RobotContainer::GetAutonomousCommand() {
  AutonHelper::AutonPtr chosen = _autoManager.GetChosenAuton();
  return chosen; 
}