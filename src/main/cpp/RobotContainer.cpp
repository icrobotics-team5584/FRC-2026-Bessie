// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include "subsystems/SubDeploy.h"
#include "subsystems/SubDrivebase.h"
#include "subsystems/SubFeeder.h"
#include "subsystems/SubHood.h"
#include "subsystems/SubIndexer.h"
#include "subsystems/SubIntake.h"
#include "subsystems/SubShooter.h"
#include "subsystems/SubTurret.h"
#include "subsystems/SubVision.h"

#include "commands/AutonCommands.h"
#include "commands/DriveCommands.h"
#include "commands/FuelCommands.h"
#include "commands/TurretCommands.h"
#include "commands/VisionCommands.h"

#include "utilities/Logger.h"
#include "utilities/PoseHandler.h"
#include "utilities/ShiftHandler.h"

#include <frc2/command/Commands.h>

RobotContainer::RobotContainer() {
  ConfigureBindings();
  SubDrivebase::GetInstance().SetDefaultCommand(cmd::TeleopDrive(_driverController));
  SubVision::GetInstance().SetDefaultCommand(cmd::AddVisionMeasurement());
  SubTurret::GetInstance().SetDefaultCommand(cmd::AimAtFieldRelative([]{return cmd::CalcShootOnTheMoveAngle();}));

  _autoManager.AddDefaultAuton("default", AutonHelper::MakeCommandPtrAuto(cmd::DefaultAuton()));

  frc::SmartDashboard::PutData("CHOSEN AUTON", &_autoManager.GetAutonChooser());

  SubHood::GetInstance();
  SubShooter::GetInstance();
}

void RobotContainer::ConfigureBindings() {
  //Triggers
  _driverController.LeftTrigger().WhileTrue(cmd::IntakeSequence());
  _driverController.RightTrigger().WhileTrue(cmd::ShootOnTheMove().AlongWith(SubDrivebase::GetInstance().Drive([] { return frc::ChassisSpeeds{0_mps, 0_mps, 1_tps}; }, false)));
  _driverController.RightTrigger().OnFalse(SubFeeder::GetInstance().FeederOff());

  //Bumpers
  _driverController.LeftBumper().ToggleOnTrue(SubDeploy::GetInstance().ToggleDeploy());
  _driverController.RightBumper().WhileTrue(SubDrivebase::GetInstance().LockWheelsInXShape());

  //Letters
  _driverController.X().WhileTrue(SubDrivebase::GetInstance().CharacteriseWheels());
  _driverController.Y().OnTrue(SubDrivebase::GetInstance().ResetGyroCmd());
  _driverController.B().WhileTrue(SubDrivebase::GetInstance().AlignToAngle(_driverController, 0_deg));
  _driverController.A().OnTrue(frc2::cmd::RunOnce([] {
    SubDrivebase::GetInstance().SetPose(frc::Pose2d{0_m, 0_m, 0_deg});
  }));

  //POVs
  _driverController.POVDown().OnTrue(
    SubTurret::GetInstance().SetTurretTargetAngle([] { return 180_deg; }, [] { return 0_deg_per_s; }));
  _driverController.POVRight().OnTrue(cmd::AimAtSpot(frc::Translation2d{0_m, 0_m}));
  _driverController.POVLeft().WhileTrue(SubHood::GetInstance().ZeroHood());

  //Sticks

  //Other

  frc2::Trigger([]{return ShiftHandler::GetTimeLeft() < 3_s;}).OnTrue(Rumble(1, 0.5_s));
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