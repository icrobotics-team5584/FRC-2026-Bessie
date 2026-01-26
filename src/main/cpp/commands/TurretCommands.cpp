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
  // Calculate distance to target **FROM TURRET**
  auto target = frc::Pose2d{0_m,0_m,0_deg};
  auto robot = PoseHandler::GetInstance().GetPose(); // add distance turret relative to robot
  Logger::FieldDisplay::GetInstance().DisplayPose("SOTM/robotPose", robot);
  units::meter_t distance = target.Translation().Distance(robot.Translation());

  // Calculate field relative turret velocity
  units::meters_per_second_t robotX = SubDrivebase::GetInstance().GetVelocityX();
  units::meters_per_second_t robotY = SubDrivebase::GetInstance().GetVelocityY();
  Logger::Log("SOTM/velX", robotX);
  Logger::Log("SOTM/velY", robotY);
  // Adjust with rotation speed and turret relative to robot

  // Account for robot velocity
  // Get future pose
  units::second_t TOF = SubShooter::GetInstance().GetTimeOfFLightWithDistance(distance);
  Logger::Log("SOTM/ToF", TOF);

  units::meter_t offsetX = robotX * TOF;
  units::meter_t offsetY = robotY * TOF;
  frc::Pose2d futurePose = frc::Pose2d(offsetX, offsetY, robot.Rotation());
  Logger::FieldDisplay::GetInstance().DisplayPose("SOTM/Future Pose", futurePose);

  // Find parameters from future pose to target
  units::meter_t futureDistance = target.Translation().Distance(futurePose.Translation());
  Logger::Log("SOTM/futureDistance", futureDistance);

  return futureDistance;
}

units::degree_t CalcShootOnTheMoveAngle() {
  // Calculate distance to target **FROM TURRET**
  auto target = frc::Pose2d{0_m,0_m,0_deg};
  auto robot = PoseHandler::GetInstance().GetPose(); // add distance turret relative to robot
  Logger::FieldDisplay::GetInstance().DisplayPose("SOTM/robotPose", robot);
  units::meter_t distance = target.Translation().Distance(robot.Translation());

  // Calculate field relative turret velocity
  units::meters_per_second_t robotVelX = SubDrivebase::GetInstance().GetVelocityX();
  units::meters_per_second_t robotVelY = SubDrivebase::GetInstance().GetVelocityY();
  units::degrees_per_second_t robotVelRot = SubDrivebase::GetInstance().GetAngularVelocity();
  Logger::Log("SOTM/velX", robotVelX);
  Logger::Log("SOTM/velY", robotVelY);
  Logger::Log("SOTM/velRot", robotVelRot);
  // Adjust with rotation speed and turret relative to robot

  // Account for robot velocity
  // Get future pose
  units::second_t TOF = SubShooter::GetInstance().GetTimeOfFLightWithDistance(distance);
  Logger::Log("SOTM/ToF", TOF);

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
  
  frc::Pose2d futurePose = frc::Pose2d(robotX - offsetX, robotY - offsetY, robot.Rotation().Degrees() - offsetRot);
  Logger::FieldDisplay::GetInstance().DisplayPose("SOTM/Future Pose", futurePose);

  // Find parameters from future pose to target

  units::radian_t angleFromFutureToTarget = atan2((target.Y() - futurePose.Y()).value(), (target.X() - futurePose.X()).value()) * 1_rad;
  units::degree_t angleFromFutureToTargetDegrees = angleFromFutureToTarget;

  Logger::Log("SOTM/CalcTurretAngle", angleFromFutureToTargetDegrees);

  return angleFromFutureToTargetDegrees;
}

frc2::CommandPtr Shoot() {
  // return SubShooter::GetInstance().SpinWithDistance( [] {return CalcShootOnTheMoveDistance();})
  // .AlongWith(SubHood::GetInstance().AimWithDistance([] {return CalcShootOnTheMoveDistance();}))
  // .AlongWith(AimAtFieldRelative([] {return CalcShootOnTheMoveAngle();}))
  // .AndThen(SubFeeder::GetInstance().FeederOn())
  // .OnlyIf([] {return SubShooter::GetInstance().IsAtSpeed() && SubTurret::GetInstance().IsAtTarget() && SubHood::GetInstance().IsAtTarget();} );
  return AimAtFieldRelative([] {return CalcShootOnTheMoveAngle();});
}

}  // namespace cmd
