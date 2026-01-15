// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/Commands.h>
#include "subsystems/SubDrivebase.h"
#include "subsystems/SubVision.h"
#include "subsystems/SubHood.h"
#include "subsystems/SubShooter.h"
#include "subsystems/SubTurret.h"
#include "commands/DriveCommands.h"
#include "commands/VisionCommand.h"

RobotContainer::RobotContainer() {
  SubVision::GetInstance();
  SubDrivebase::GetInstance().SetDefaultCommand(cmd::TeleopDrive(_driverController));
  SubVision::GetInstance().SetDefaultCommand(cmd::AddVisionMeasurement());
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
  _driverController.X().OnTrue(SubHood::GetInstance().SetHoodPosition(0_deg));
  _driverController.Y().OnTrue(SubHood::GetInstance().SetHoodPosition(20_deg));
  _driverController.A().OnTrue(SubTurret::GetInstance().SetTurretAngle(0_deg));
  _driverController.A().OnTrue(SubTurret::GetInstance().SetTurretAngle(20_deg));
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return frc2::cmd::Print("No autonomous command configured");
}
