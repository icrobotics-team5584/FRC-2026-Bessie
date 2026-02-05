// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include <frc/DriverStation.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Translation3d.h>
class ShotPlanner {
 public:
  struct ShotPlannerResults {
    frc::Translation3d targetPosition;
    bool shouldShoot;
  };

  ShotPlanner();
  static ShotPlannerResults CalculateShotTarget(frc::Pose2d robotPos);
  static bool IsWithinZone(
    frc::Translation2d topRightCorner, frc::Translation2d bottomLeftCorner, frc::Pose2d robotPos);
  static frc::Pose2d ConvertToPose2d(frc::Translation3d translation3d);
};
