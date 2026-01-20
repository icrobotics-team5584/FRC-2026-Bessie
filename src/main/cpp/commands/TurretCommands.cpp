#include "commands/TurretCommands.h"

#include "subsystems/SubHood.h"
#include "subsystems/SubShooter.h"
#include "subsystems/SubTurret.h"

#include "utilities/Logger.h"
#include "utilities/PoseHandler.h"

#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>

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

frc2::CommandPtr ShootOnTheMove(frc::Pose3d ShooterTarget) {
  
  units::meters_per_second_t exitVelocity = 3_mps;
  units::meters_per_second_squared_t g = 9.81_mps_sq;

  units::meter_t launchHeight = 0.5_m;
  units::meter_t deltaHeight = ShooterTarget.Z() - launchHeight;

  auto theta = SubHood::GetInstance().GetHoodAngle();

  auto exitVelocityY = exitVelocity * units::math::sin(theta);

  // discriminant = (exitVelocityY)^2 - 2gh
  auto discriminant = exitVelocityY * exitVelocityY - 2 * g * deltaHeight;

  units::second_t travelTime = (exitVelocityY + units::math::sqrt(discriminant)) / g;

  Logger::Log("Shooter/fuelTravelTime", travelTime);
  Logger::Log("Shooter/calculatedDiscriminant", discriminant.value());

  return cmd::AimAtPose(SubDrivebase::GetInstance().PredictPose(travelTime));
};
}  // namespace cmd
