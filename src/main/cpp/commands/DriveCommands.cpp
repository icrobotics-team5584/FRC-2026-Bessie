// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/DriveCommands.h"
#include "commands/FuelCommands.h"
#include <frc2/command/Commands.h>
#include "subsystems/SubDrivebase.h"
#include "subsystems/SubVision.h"
#include <frc/DriverStation.h>

namespace cmd {
using namespace frc2::cmd;

frc2::CommandPtr TeleopDrive(frc2::CommandXboxController& controller) {
  return SubDrivebase::GetInstance().Drive([&controller] {
    auto speeds = SubDrivebase::GetInstance().CalcJoystickSpeeds(controller);
    if (cmd::IsShooting()) {
      speeds.vx = std::clamp(speeds.vx, -2.5_mps, 2.5_mps);
      speeds.vy = std::clamp(speeds.vy, -2.5_mps, 2.5_mps);
      speeds.omega = std::clamp(speeds.omega, -1.57_rad_per_s, 1.57_rad_per_s);
    }
    return frc::ChassisSpeeds{speeds.vx, speeds.vy, speeds.omega};
  }, true);
}
}  // namespace cmd
