#include "commands/TurretCommands.h"

#include "subsystems/SubHood.h"
#include "subsystems/SubShooter.h"
#include "subsystems/SubTurret.h"
#include "subsystems/SubFeeder.h"
#include "subsystems/SubDrivebase.h"
#include "subsystems/SubIndexer.h"
#include "subsystems/SubIntake.h"

#include "utilities/Logger.h"
#include "utilities/PoseHandler.h"

#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>

namespace cmd {  
   
  frc2::CommandPtr AimAtFieldRelative(std::function<units::degree_t()> target) {
    return SubTurret::GetInstance().SetTurretTargetAngle([target] {     
      auto robotPose = PoseHandler::GetInstance().GetPose();
      Logger::Log("Turret/AimAtFieldRelative/robotPose/Rotation", robotPose.Rotation().Degrees());
      units::degree_t targetAngle = target() - robotPose.Rotation().Degrees() - 180_deg;
      return targetAngle;});
  }

  frc2::CommandPtr AimAtSpot(frc::Translation2d target) {
    return cmd::AimAtFieldRelative([target] {
      auto robotPose = PoseHandler::GetInstance().GetPose();
      units::radian_t angle = atan2( (target.Y()-robotPose.Y()).value(), (target.X()-robotPose.X()).value() ) * 1_rad;
      units::degree_t degrees = angle;
      return degrees;
    });
  }

units::meter_t CalcShootOnTheMoveDistance() {
  auto target = frc::Pose2d{4.65_m, 4_m, 0_deg};;

  frc::Pose2d futurePose = CalcFuturePose();

  // Find parameters from future pose to target
  units::meter_t futureDistance = target.Translation().Distance(futurePose.Translation());
  Logger::Log("SOTM/futureDistance", futureDistance);

  return futureDistance;
}

units::degree_t CalcShootOnTheMoveAngle() {
  auto target = frc::Pose2d{4.65_m, 4_m, 0_deg};;
  frc::Pose2d futurePose = CalcFuturePose();

  // Find angle from future pose to target
  units::radian_t angleFromFutureToTarget = atan2((target.Y() - futurePose.Y()).value(), (target.X() - futurePose.X()).value()) * 1_rad;
  units::degree_t angleFromFutureToTargetDegrees = angleFromFutureToTarget;

  Logger::Log("SOTM/CalcTurretAngle", angleFromFutureToTargetDegrees);

  return angleFromFutureToTargetDegrees;
}

frc2::CommandPtr ShootWhenReady() {
  return frc2::cmd::Either(SubFeeder::GetInstance().FeederOn().AlongWith(SubIndexer::GetInstance().Index()),
    SubFeeder::GetInstance().FeederOff().AlongWith(SubIndexer::GetInstance().StopIndex()),
    [] {
      return SubHood::GetInstance().HoodIsAtTarget() && SubShooter::GetInstance().IsAtSpeed() &&
             SubTurret::GetInstance().IsAtTarget();
    })
    .Repeatedly();
}

frc2::CommandPtr AimOnTheMove() {
  return SubShooter::GetInstance().SpinWithDistance( [] {return CalcShootOnTheMoveDistance();})
  .AlongWith(SubHood::GetInstance().SetHoodPositionTargetFromDist([] {return CalcShootOnTheMoveDistance();}))
  .AlongWith(AimAtFieldRelative([] {return CalcShootOnTheMoveAngle();}));
}

frc2::CommandPtr ShootOnTheMove() {
  return AimOnTheMove().AlongWith(ShootWhenReady()).AlongWith(SubIntake::GetInstance().IntakeOn());
}

frc::Pose2d CalcFuturePose() {
  // Calculate distance to target **FROM TURRET**
  auto target = frc::Pose2d{4.65_m, 4_m, 0_deg};
  auto robot = PoseHandler::GetInstance().GetPose(); // add distance turret relative to robot
  Logger::FieldDisplay::GetInstance().DisplayPose("SOTM/robotPose", robot);
  units::meter_t distance = target.Translation().Distance(robot.Translation());

  // Calculate field relative turret velocity
  frc::ChassisSpeeds robotVel = SubDrivebase::GetInstance().GetFieldRelativeVelocity();
  units::meters_per_second_t robotVelX = robotVel.vx;
  units::meters_per_second_t robotVelY = robotVel.vy;
  units::degrees_per_second_t robotVelRot = SubDrivebase::GetInstance().GetAngularVelocity();
  
  Logger::Log("SOTM/velX", robotVelX);
  Logger::Log("SOTM/velY", robotVelY);
  Logger::Log("SOTM/velRot", robotVelRot);

  // Account for robot velocity
  // Get future pose
  units::second_t TOF = SubShooter::GetInstance().GetTimeOfFLightWithDistance(distance);
  Logger::Log("SOTM/ToF", TOF);

  // Account for robot acceleration

  units::meter_t offsetX = robotVelX * TOF;
  units::meter_t offsetY = robotVelY * TOF;
  units::degree_t offsetRot = robotVelRot * TOF;
  units::degree_t robotRotation = robot.Rotation().Degrees();
  units::meter_t robotX = robot.X();
  units::meter_t robotY = robot.Y();

  Logger::Log("SOTM/robotX", robotX);
  Logger::Log("SOTM/robotY", robotY);
  Logger::Log("SOTM/robotRotation", robotRotation);

  Logger::Log("SOTM/offsetX", offsetX);
  Logger::Log("SOTM/offsetY", offsetY);
  Logger::Log("SOTM/offsetRot", offsetRot);
  
  frc::Pose2d futurePose = frc::Pose2d(robotX + offsetX, robotY + offsetY, robot.Rotation().Degrees() + offsetRot);
  
  Logger::FieldDisplay::GetInstance().DisplayPose("SOTM/futurePose", futurePose);

  frc::Pose2d turretFuturePose = futurePose.TransformBy(SubTurret::ROBOT_TO_TURRET);

  Logger::FieldDisplay::GetInstance().DisplayPose("SOTM/futureTurretPose", turretFuturePose);
  return turretFuturePose;
}

}  // namespace cmd
