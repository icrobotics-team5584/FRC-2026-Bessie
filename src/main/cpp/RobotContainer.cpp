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

#include "utilities/PoseHandler.h"

#include <frc2/command/Commands.h>

RobotContainer::RobotContainer() {
  SubDrivebase::GetInstance().SetDefaultCommand(cmd::TeleopDrive(_driverController));
  ConfigureBindings();
  SubVision::GetInstance().SetDefaultCommand(cmd::AddVisionMeasurement());

  _autoManager.AddDefaultAuton("default", AutonHelper::MakeCommandPtrAuto(cmd::DefaultAuton()));

  frc::SmartDashboard::PutData("CHOSEN AUTON", &_autoManager.GetAutonChooser());

  SubTurret::GetInstance();
  SubHood::GetInstance();
  SubShooter::GetInstance();
}

void RobotContainer::ConfigureBindings() {
  //Triggers
  _driverController.LeftTrigger().WhileTrue(cmd::IntakeSequence());

  //Bumpers
  _driverController.LeftBumper().ToggleOnTrue(SubDeploy::GetInstance().ToggleDeploy());
  _driverController.RightBumper().OnTrue(SubVision::GetInstance().CalibrateRobotToCamera(
    frc::Transform3d{frc::Translation3d{1_m, 0_m, 1.12395_m},
      frc::Rotation3d{0_deg, 0_deg, 180_deg}}));  // 1m away from april tag

  //Letters
  _driverController.X().WhileTrue(SubDrivebase::GetInstance().CharacteriseWheels());
  _driverController.Y().OnTrue(SubDrivebase::GetInstance().ResetGyroCmd());
  _driverController.B().OnTrue(SubDrivebase::GetInstance().SyncSensor());
  _driverController.A().OnTrue(frc2::cmd::RunOnce([] {
    SubDrivebase::GetInstance().SetPose(frc::Pose2d{3.0218614_m, 4.3902376_m, 0_deg}); //1m away from april tag 25
  }));

  //POVs
  _driverController.POVUp().OnTrue(cmd::AimAtFieldRelative([] { return 0_deg; }));
  _driverController.POVDown().OnTrue(
    SubTurret::GetInstance().SetTurretTargetAngle([] { return 0_deg; }));
  _driverController.POVRight().OnTrue(cmd::AimAtPose(frc::Pose2d{0_m, 0_m, 0_deg}));

  //Other

}

std::shared_ptr<frc2::CommandPtr> RobotContainer::GetAutonomousCommand() {
  AutonHelper::AutonPtr chosen = _autoManager.GetChosenAuton();
  return chosen;
}