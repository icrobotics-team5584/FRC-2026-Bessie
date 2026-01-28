#include "commands/FuelCommands.h"

#include "subsystems/SubDeploy.h"
#include "subsystems/SubFeeder.h"
#include "subsystems/SubHood.h"
#include "subsystems/SubIndexer.h"
#include "subsystems/SubIntake.h"
#include "subsystems/SubShooter.h"
#include "subsystems/SubTurret.h"
#include "utilities/Logger.h"

#include "commands/TurretCommands.h"

#include "utilities/PoseHandler.h"
#include <frc/geometry/Transform2d.h>

namespace cmd {
frc2::CommandPtr IntakeSequence() {
  return SubDeploy::GetInstance()
    .DeployIntake()
    .AlongWith(SubIntake::GetInstance().IntakeOn());
}

frc2::CommandPtr StationaryAimAt(frc::Translation2d target) {
  auto distanceToTarget = [target] {
    auto curPose = PoseHandler::GetInstance().GetPose();
    auto turretPose = curPose.TransformBy(SubTurret::ROBOT_TO_TURRET);

    Logger::FieldDisplay::GetInstance().DisplayPose("Turret/turretPose", turretPose);
    Logger::Log("Shooter/distToTargetInner", target.Distance(turretPose.Translation()));

    return target.Distance(turretPose.Translation());};

  return frc2::cmd::Parallel(cmd::AimAtSpot(target),
    SubShooter::GetInstance().SetShooterTargetFromDist(distanceToTarget),
    SubHood::GetInstance().SetHoodPositionTargetFromDist(distanceToTarget));
}

frc2::CommandPtr StationaryShootWhenReady() {
  return frc2::cmd::Either(SubFeeder::GetInstance().FeederOn().AlongWith(SubIndexer::GetInstance().Index()),
    SubFeeder::GetInstance().FeederOff().AlongWith(SubIndexer::GetInstance().StopIndex()),
    [] {
      return SubHood::GetInstance().HoodIsAtTarget() && SubShooter::GetInstance().IsAtSpeed() &&
             SubTurret::GetInstance().TurretIsAtTarget();
    })
    .Repeatedly();
}

}  // namespace cmd