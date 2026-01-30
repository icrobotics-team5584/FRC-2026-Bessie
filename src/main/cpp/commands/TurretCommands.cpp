#include "commands/TurretCommands.h"
#include "subsystems/SubTurret.h"
#include "subsystems/SubHood.h"
#include "subsystems/SubShooter.h"
#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>
#include "utilities/PoseHandler.h"
#include "utilities/Logger.h"

namespace cmd {   
  frc2::CommandPtr AimAtFieldRelative(std::function<units::degree_t()> target) {
    return SubTurret::GetInstance().SetTurretTargetAngle([target] {     
      auto robotPose = PoseHandler::GetInstance().GetPose();
      Logger::Log("Turret/AimAtFieldRelative/robotPose/Rotation", robotPose.Rotation().Degrees());
      units::degree_t targetAngle = target() - robotPose.Rotation().Degrees();
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
}
