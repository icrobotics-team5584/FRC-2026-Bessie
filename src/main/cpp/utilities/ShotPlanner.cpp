// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "utilities/ShotPlanner.h"

#include <utilities/ICgeometry.h>
#include <utilities/Logger.h>

ShotPlanner::ShotPlanner() = default;

frc::Translation3d ShotPlanner::CalculateShotTarget(frc::Pose2d robotPos) {
  auto alliance = frc::DriverStation::GetAlliance();
  if (alliance) {
    if (alliance.value() == frc::DriverStation::kRed) {
      robotPos = ICgeometry::xPoseFlip(robotPos);
    }
    Logger::FieldDisplay::GetInstance().DisplayPose("Shot Planner/Robot Position", robotPos);
  }

  frc::Translation2d blueAllianceZoneTopRight{4_m, 8_m};
  frc::Translation2d blueAllianceZoneBottomLeft{0_m, 0_m};

  frc::Translation2d topZoneTopRight{16.5_m, 8_m};
  frc::Translation2d topZoneBottomLeft{4_m, 4_m};

  frc::Translation2d bottomZoneTopRight{16.5_m, 4_m};
  frc::Translation2d bottomZoneBottomLeft{4_m, 0_m};

  frc::Translation2d hubPosition{4_m, 4_m};

  frc::Translation2d topAllianceZonePosition{2.5_m, 5.5_m};
  frc::Translation2d bottomAllianceZonePosition{2.5_m, 2.75_m};

  frc::Translation3d target;

  if (IsWithinZone(blueAllianceZoneTopRight, blueAllianceZoneBottomLeft, robotPos)) {
    target = frc::Translation3d(hubPosition.X(), hubPosition.Y(), 1.83_m);
  } else if (IsWithinZone(topZoneTopRight, topZoneBottomLeft, robotPos)) {
    target = frc::Translation3d(topAllianceZonePosition);
  } else if (IsWithinZone(bottomZoneTopRight, bottomZoneBottomLeft, robotPos)) {
    target = frc::Translation3d(bottomAllianceZonePosition);
  } else {
    target = frc::Translation3d(hubPosition.X(), hubPosition.Y(), 1.83_m);
  }

  if (alliance) {
    if (alliance.value() == frc::DriverStation::Alliance::kRed) {
      target = ICgeometry::xTranslationFlip(target);
    }
  }

  return target;
}

bool ShotPlanner::IsWithinZone(
  frc::Translation2d corner1, frc::Translation2d corner2, frc::Pose2d robotPos) {
  if (robotPos.X() < corner1.X() && robotPos.X() > corner2.X()) {
    if (robotPos.Y() < corner1.Y() && robotPos.Y() > corner2.Y()) {
      return true;
    }
  }
  return false;
}

frc::Pose2d ShotPlanner::ConvertToPose2d(frc::Translation3d translation3d) {
  frc::Translation2d translation2d = translation3d.ToTranslation2d();
  frc::Pose2d targetPose{translation2d, 0_deg};
  return targetPose;
}
