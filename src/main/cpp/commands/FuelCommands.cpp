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
    .AlongWith(SubIntake::GetInstance().IntakeOn())
    .AlongWith(SubIndexer::GetInstance().IndexerOn());
}

frc2::CommandPtr StationaryShootAt(frc::Pose2d target) {
  auto distanceToTarget = [target] {
    auto curPose = PoseHandler::GetInstance().GetPose();
    auto turretPose = curPose.TransformBy(frc::Transform2d{-235_mm, 0_mm, 0_deg});

    Logger::FieldDisplay::GetInstance().DisplayPose("Turret/turretPose", turretPose);
    Logger::Log("Shooter/distToTargetInner", target.Translation().Distance(turretPose.Translation()));

    return target.Translation().Distance(turretPose.Translation());};

  return frc2::cmd::Parallel(cmd::AimAtPose(target),
    SubShooter::GetInstance().SetShooterTargetFromDist(distanceToTarget),
    SubHood::GetInstance().SetHoodPositionTargetFromDist(distanceToTarget))
    .Until([] {
      return SubShooter::GetInstance().IsAtSpeed() && SubTurret::GetInstance().TurretIsAtTarget() &&
             SubHood::GetInstance().HoodIsAtTarget();
    })
    .AndThen(frc2::cmd::Parallel(SubIntake::GetInstance().IntakeOn(),
      SubFeeder::GetInstance().FeederOn(), SubIndexer::GetInstance().Index()));
}

}  // namespace cmd