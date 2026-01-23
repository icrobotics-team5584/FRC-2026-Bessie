// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include <frc/geometry/Translation3d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Pose2d.h>
class ShotPlanner {
 public:
  ShotPlanner();
  frc::Translation3d CalculateShotTarget(frc::Pose2d robotPos);
  bool IsWithinZone(frc::Translation2d corner1, frc::Translation2d corner2, frc::Pose2d robotPos);
};
