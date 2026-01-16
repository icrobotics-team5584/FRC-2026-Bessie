#include "commands/TurretCommands.h"

#include "subsystems/SubHood.h"
#include "subsystems/SubShooter.h"
#include "subsystems/SubTurret.h"

#include "utilities/PoseHandler.h"

#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>

namespace cmd {
frc2::CommandPtr AimAt(frc::Pose2d target) {
  return SubTurret::GetInstance().SetTurretTargetAngle([target] {
    auto robotPose = PoseHandler::GetInstance().GetPose();
    auto diff = target.RelativeTo(robotPose);
    return diff.Rotation().Degrees();
  });
}
}  // namespace cmd