// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include "subsystems/Hood/SubHood.h"
#include "subsystems/SubClimber.h"
#include "subsystems/SubDeploy.h"
#include "subsystems/SubDrivebase.h"
#include "subsystems/SubFeeder.h"
#include "subsystems/SubIndexer.h"
#include "subsystems/SubIntake.h"
#include "subsystems/SubShooter.h"
#include "subsystems/SubVision.h"
#include "subsystems/Turret/SubTurret.h"

#include "commands/AutonCommands.h"
#include "commands/DriveCommands.h"
#include "commands/FuelCommands.h"
#include "commands/TurretCommands.h"
#include "commands/VisionCommands.h"

#include "utilities/Logger.h"
#include "utilities/PoseHandler.h"
#include "utilities/ShiftHandler.h"
#include "utilities/ShotPlanner.h"

#include <frc2/command/Commands.h>

RobotContainer::RobotContainer() {
  ConfigureBindings();
  SubDrivebase::GetInstance().SetDefaultCommand(cmd::TeleopDrive(_driverController));
  SubVision::GetInstance().SetDefaultCommand(cmd::AddVisionMeasurement());
  SubTurret::GetInstance().SetDefaultCommand(cmd::AimAtHub());

  _autoManager.AddDefaultAuton("ShootAndStay", AutonHelper::MakeCommandPtrAuto(cmd::ShootAndStay()));

  _autoManager.AddAuton("DriveInASquare",
    AutonHelper::MakeCommandPtrAuto(cmd::TESTDriveInASquare()));
  _autoManager.AddAuton("Forward250cm",
    AutonHelper::MakeCommandPtrAuto(cmd::TESTForward250cm()));
  _autoManager.AddAuton("Forward250cmWhileTurning",
    AutonHelper::MakeCommandPtrAuto(cmd::TESTForward250cmWhileTurning()));

  // _autoManager.AddAuton("NeutralOnePassAndClimb_LeftBump",
  //   AutonHelper::MakeCommandPtrAuto(cmd::NeutralOnePassAndClimb_LeftBump()));
  // _autoManager.AddAuton("NeutralOnePassAndClimb_LeftTrench",
  //   AutonHelper::MakeCommandPtrAuto(cmd::NeutralOnePassAndClimb_LeftTrench()));
  // _autoManager.AddAuton("NeutralOnePassAndClimb_LeftTrench_NoBump",
  //   AutonHelper::MakeCommandPtrAuto(cmd::NeutralOnePassAndClimb_LeftTrench_NoBump()));

  // _autoManager.AddAuton("NeutralOnePassAndClimb_RightBump",
  //   AutonHelper::MakeCommandPtrAuto(cmd::NeutralOnePassAndClimb_RightBump()));
  // _autoManager.AddAuton("NeutralOnePassAndClimb_RightTrench",
  //   AutonHelper::MakeCommandPtrAuto(cmd::NeutralOnePassAndClimb_RightTrench()));
  // _autoManager.AddAuton("NeutralOnePassAndClimb_RightTrench_NoBump",
  //   AutonHelper::MakeCommandPtrAuto(cmd::NeutralOnePassAndClimb_RightTrench_NoBump()));

  _autoManager.AddAuton("Hoard_LeftBump",
    AutonHelper::MakeCommandPtrAuto(cmd::Hoard_LeftBump()));
  _autoManager.AddAuton("Hoard_LeftTrench",
    AutonHelper::MakeCommandPtrAuto(cmd::Hoard_LeftTrench()));
  _autoManager.AddAuton("Hoard_RightBump",
    AutonHelper::MakeCommandPtrAuto(cmd::Hoard_RightBump()));
  _autoManager.AddAuton("Hoard_RightTrench",
    AutonHelper::MakeCommandPtrAuto(cmd::Hoard_RightTrench()));

  _autoManager.AddAuton("NeutralOnePassAndOutpost_RightBump",
    AutonHelper::MakeCommandPtrAuto(cmd::NeutralOnePassAndOutpost_RightBump()));
  _autoManager.AddAuton("NeutralOnePassAndOutpost_RightTrench",
    AutonHelper::MakeCommandPtrAuto(cmd::NeutralOnePassAndOutpost_RightTrench()));
  _autoManager.AddAuton("NeutralOnePassAndOutpost_RightTrench_NoBump",
    AutonHelper::MakeCommandPtrAuto(cmd::NeutralOnePassAndOutpost_RightTrench_NoBump()));

  // _autoManager.AddAuton("NeutralOnePassOutpostClimb_RightTrench", 
  //   AutonHelper::MakeCommandPtrAuto(cmd::GASTAUTON_NeutralOnePassOutpostClimb_RightTrench()));
  // _autoManager.AddAuton("NeutralOnePassOutpostClimb_RightTrench_NoBump",
  //   AutonHelper::MakeCommandPtrAuto(cmd::GASTAUTON_NeutralOnePassOutpostClimb_RightTrench_NoBump()));

  frc::SmartDashboard::PutData("CHOSEN AUTON", &_autoManager.GetAutonChooser());

  SubHood::GetInstance();
  SubShooter::GetInstance();
}

void RobotContainer::ConfigureBindings() {
  //Triggers
  _driverController.LeftTrigger().WhileTrue(cmd::IntakeSequence());
  _driverController.RightTrigger().WhileTrue(cmd::ShootOnTheMove().AlongWith(cmd::TeleopDrive(_driverController, 1.0)));
  _driverController.RightTrigger().OnFalse(SubFeeder::GetInstance().FeederOff());

  //Bumpers
  _driverController.LeftBumper().ToggleOnTrue(SubDeploy::GetInstance().ToggleDeploy());
  _driverController.RightBumper().WhileTrue(SubDrivebase::GetInstance().LockWheelsInXShape());

  //Letters
  _driverController.X().WhileTrue(SubDrivebase::GetInstance().CharacteriseWheels());
  _driverController.B().WhileTrue(SubDrivebase::GetInstance().AlignToAngle(_driverController, 0_deg));
  _driverController.A().WhileTrue(cmd::EjectFuel());
  _driverController.Y().ToggleOnTrue(SubClimber::GetInstance().ToggleClimb());

  // Misc 
  _driverController.Start().OnTrue(SubDrivebase::GetInstance().ZeroRotation());

  /* Operator */
  _operatorController.Back().OnTrue(frc2::cmd::RunOnce([]{ ShiftHandler::GetInstance().SetOverrideActive(true); }));
  _operatorController.X().OnTrue(frc2::cmd::RunOnce([]{ ShiftHandler::GetInstance().SetOverrideActive(true); }));
  _operatorController.X().OnFalse(frc2::cmd::RunOnce([]{ ShiftHandler::GetInstance().SetOverrideActive(false); }));
  _operatorController.Y().OnTrue(cmd::DisableAllOverrides());
  _operatorController.RightTrigger().WhileTrue(cmd::BackupShoot());
  _operatorController.Start().OnTrue(cmd::ForceShoot());

  _operatorController.LeftBumper().OnTrue(frc2::cmd::RunOnce([]{return ShotPlanner::SetOverride(ShotPlanner::Override::SCORE);})
  .AlongWith(frc2::cmd::RunOnce([]{ ShiftHandler::GetInstance().SetOverrideActive(true); })));
  _operatorController.RightBumper().OnTrue(frc2::cmd::RunOnce([]{return ShotPlanner::SetOverride(ShotPlanner::Override::PASS);})
  .AlongWith(frc2::cmd::RunOnce([]{ ShiftHandler::GetInstance().SetOverrideActive(true); })));

  // Operator POVS
  _operatorController.POVRight().OnTrue(SubShooter::GetInstance().AdjustManualSpeedOffset(1_tps));
  _operatorController.POVLeft().OnTrue(SubShooter::GetInstance().AdjustManualSpeedOffset(-1_tps));
  _operatorController.POVUp().OnTrue(SubHood::GetInstance().AdjustManualAngleOffset(1_deg));
  _operatorController.POVDown().OnTrue(SubHood::GetInstance().AdjustManualAngleOffset(-1_deg));

  // Driver POVs
  _driverController.POVRight().WhileTrue(SubDeploy::GetInstance().DeployAutoZero());
  _driverController.POVLeft().WhileTrue(SubHood::GetInstance().ZeroHood());
  _driverController.POVUp().WhileTrue(SubClimber::GetInstance().RunCurrentZeroingSequence());

  //Sticks

  //Other
  _driverController.Back().OnTrue(SubDrivebase::GetInstance().SyncSensor());

  frc2::Trigger([]{return ShiftHandler::GetInstance().GetTimeLeft() < 3_s;}).OnTrue(Rumble(1, 0.5_s));
  SubDrivebase::GetInstance().CheckCoastButton().ToggleOnTrue(cmd::ToggleBrakeCoast());
}

std::shared_ptr<frc2::CommandPtr> RobotContainer::GetAutonomousCommand() {
  AutonHelper::AutonPtr chosen = _autoManager.GetChosenAuton();
  return chosen;
}

frc2::CommandPtr RobotContainer::Rumble(double force, units::second_t duration) {
  return frc2::cmd::Run([this, force, duration] {
    _driverController.SetRumble(frc::XboxController::RumbleType::kBothRumble, force);
    Logger::Log("DriverStation/Rumble", true);
  })
    .WithTimeout(duration)
    .FinallyDo([this] {
      _driverController.SetRumble(frc::XboxController::RumbleType::kBothRumble, 0);
      Logger::Log("DriverStation/Rumble", false);
    });
}