// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "utilities/ShotPlanner.h"

#include "utilities/FieldConstants.h"
#include "utilities/ShiftHandler.h"

#include <utilities/ICgeometry.h>
#include <utilities/Logger.h>

ShotPlanner::ShotPlanner() = default;

ShotPlanner::ShotPlannerResults ShotPlanner::CalculateShotTarget(frc::Pose2d robotPos) {
  auto alliance = frc::DriverStation::GetAlliance();
  if (alliance) {
    if (alliance.value() == frc::DriverStation::kRed) {
      robotPos = ICgeometry::xPoseFlip(robotPos);
    }
    Logger::FieldDisplay::GetInstance().DisplayPose("Shot Planner/Robot Position", robotPos);
  }

  frc::Translation3d target;
  bool shouldShoot = true;
  bool isInBotNeutralZone = false;
  bool isInBlueAlliance = false;
  bool isOurHubActive = ShiftHandler::IsActiveShift();

  isInBlueAlliance = IsWithinZone(
    fieldpos::BLUE_ALLIANCE_ZONE_TOP_RIGHT, fieldpos::BLUE_ALLIANCE_ZONE_BOTTOM_LEFT, robotPos);
  isInBotNeutralZone = IsWithinZone(
    fieldpos::BOTTOM_PASSING_ZONE_TOP_RIGHT, fieldpos::BOTTOM_PASSING_ZONE_BOTTOM_LEFT, robotPos);

  if (isInBlueAlliance && !isOurHubActive) {
    shouldShoot = false;
  }

  if (isInBlueAlliance && isOurHubActive) {
    target = fieldpos::HUB_POSITION;
  } else {                    /* Can't score in neutral zone */
    if (isInBotNeutralZone) { /* bottom neutral zone */
      target = fieldpos::BOTTOM_ALLIANCE_ZONE_POSITION;
    } else { /* Top neutral zone */
      target = fieldpos::TOP_ALLIANCE_ZONE_POSITION;
    }
  }

  if (alliance) {
    if (alliance.value() == frc::DriverStation::Alliance::kRed) {
      target = ICgeometry::xTranslationFlip(target);
    }
  }

  return {target, shouldShoot};
}

bool ShotPlanner::IsWithinZone(
  frc::Translation2d topRightCorner, frc::Translation2d bottomLeftCorner, frc::Pose2d robotPos) {
  if (robotPos.X() < topRightCorner.X() && robotPos.X() > bottomLeftCorner.X()) {
    if (robotPos.Y() < topRightCorner.Y() && robotPos.Y() > bottomLeftCorner.Y()) {
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
