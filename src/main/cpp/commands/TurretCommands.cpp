#include "commands/TurretCommands.h"

#include "subsystems/SubHood.h"
#include "subsystems/SubShooter.h"
#include "subsystems/SubTurret.h"
#include "subsystems/SubFeeder.h"
#include "subsystems/SubDrivebase.h"

#include "utilities/Logger.h"
#include "utilities/PoseHandler.h"

#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>

#include <frc/geometry/Pose2d.h>

namespace cmd {
frc2::CommandPtr AimAtFieldRelative(std::function<units::degree_t()> target) {
  return SubTurret::GetInstance().SetTurretTargetAngle([target] {
    auto robotPose = PoseHandler::GetInstance().GetPose();
    Logger::Log("Turret/AimAtFieldRelative/robotPose/Rotation", robotPose.Rotation().Degrees());
    units::degree_t targetAngle = target() - robotPose.Rotation().Degrees();
    return targetAngle;
  });
}

frc2::CommandPtr AimAtPose(frc::Pose2d pose) {
  return cmd::AimAtFieldRelative([pose] {
    auto robotPose = PoseHandler::GetInstance().GetPose();
    units::radian_t angle =
      atan2((pose.Y() - robotPose.Y()).value(), (pose.X() - robotPose.X()).value()) * 1_rad;
    units::degree_t degrees = angle;
    return degrees;
  });
}

units::meter_t CalcShootOnTheMoveDistance() {
  // Calculate distance to target **FROM TURRET**
  auto target = frc::Pose2d{0_m,0_m,0_deg};
  auto robot = PoseHandler::GetInstance().GetPose(); // add distance turret relative to robot
  units::meter_t distance = target.Translation().Distance(robot.Translation());

  // Calculate field relative turret velocity
  units::meters_per_second_t robotX = SubDrivebase::GetInstance().GetVelocityX();
  units::meters_per_second_t robotY = SubDrivebase::GetInstance().GetVelocityY();
  // Adjust with rotation speed and turret relative to robot

  // Account for robot velocity
  // Get future pose
  units::second_t TOF = SubShooter::GetInstance().GetTimeOfFLightWithDistance(distance);
  units::meter_t offsetX = robotX * TOF;
  units::meter_t offsetY = robotY * TOF;
  frc::Pose2d futurePose = frc::Pose2d(offsetX, offsetY, robot.Rotation());

  // Find parameters from future pose to target
  units::meter_t futureDistance = target.Translation().Distance(futurePose.Translation());
  units::radian_t angleFromFutureToTarget = atan2((target.Y() - futurePose.Y()).value(), (target.X() - futurePose.X()).value()) * 1_rad;
  units::degree_t angleFromFutureToTargetDegrees = angleFromFutureToTarget;

  return futureDistance;
}

units::degree_t CalcShootOnTheMoveAngle() {
  // Calculate distance to target **FROM TURRET**
  auto target = frc::Pose2d{0_m,0_m,0_deg};
  auto robot = PoseHandler::GetInstance().GetPose(); // add distance turret relative to robot
  units::meter_t distance = target.Translation().Distance(robot.Translation());

  // Calculate field relative turret velocity
  units::meters_per_second_t robotX = SubDrivebase::GetInstance().GetVelocityX();
  units::meters_per_second_t robotY = SubDrivebase::GetInstance().GetVelocityY();
  // Adjust with rotation speed and turret relative to robot

  // Account for robot velocity
  // Get future pose
  units::second_t TOF = SubShooter::GetInstance().GetTimeOfFLightWithDistance(distance);
  units::meter_t offsetX = robotX * TOF;
  units::meter_t offsetY = robotY * TOF;
  frc::Pose2d futurePose = frc::Pose2d(offsetX, offsetY, robot.Rotation());

  // Find parameters from future pose to target
  units::meter_t futureDistance = target.Translation().Distance(futurePose.Translation());
  units::radian_t angleFromFutureToTarget = atan2((target.Y() - futurePose.Y()).value(), (target.X() - futurePose.X()).value()) * 1_rad;
  units::degree_t angleFromFutureToTargetDegrees = angleFromFutureToTarget;

  return angleFromFutureToTargetDegrees;
}

frc2::CommandPtr Shoot() {
  return SubShooter::GetInstance().SpinWithDistance(CalcShootOnTheMoveDistance)
  .AlongWith(SubHood::GetInstance().AimWithDistance(CalcShootOnTheMoveDistance))
  .AlongWith(AimAtFieldRelative(CalcShootOnTheMoveAngle))
  .AlongWith(SubFeeder::GetInstance().FeederOn())
  .OnlyIf([] {return SubShooter::GetInstance().IsAtSpeed() && SubTurret::GetInstance().IsAtTarget() && SubHood::GetInstance().IsAtTarget();} );
}

}  // namespace cmd
