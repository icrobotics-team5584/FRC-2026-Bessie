// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "utilities/ShotPlanner.h"

ShotPlanner::ShotPlanner() = default;

frc::Translation3d CalculateShotTarget(frc::Pose2d robotPos) {
    frc::Translation2d allianceZoneTopLeft {0_m,8_m};
}

bool IsWithinZone(frc::Translation2d corner1, frc::Translation2d corner2, frc::Pose2d robotPos) {
  if (robotPos.X() < corner1.X() && robotPos.X() > corner2.X()) {
    if (robotPos.Y() < corner1.Y() && robotPos.Y() > corner2.Y()) {
      return true;
    }
  }
  return false;
}
