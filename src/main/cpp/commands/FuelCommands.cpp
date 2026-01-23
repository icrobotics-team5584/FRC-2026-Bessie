#include "commands/FuelCommands.h"

#include "subsystems/SubDeploy.h"
#include "subsystems/SubFeeder.h"
#include "subsystems/SubHood.h"
#include "subsystems/SubIndexer.h"
#include "subsystems/SubIntake.h"
#include "subsystems/SubShooter.h"
#include "subsystems/SubTurret.h"

#include "commands/TurretCommands.h"

#include "utilities/PoseHandler.h"

namespace cmd {
frc2::CommandPtr IntakeSequence() {
  return SubDeploy::GetInstance()
    .DeployIntake()
    .AlongWith(SubIntake::GetInstance().IntakeOn())
    .AlongWith(SubIndexer::GetInstance().IndexerOn());
}

frc2::CommandPtr StationaryShootAt(frc::Pose2d target) {
  frc::Pose2d currentPose = PoseHandler::GetInstance().GetPose();

  units::meter_t distanceToTarget = target.Translation().Distance(currentPose.Translation());

  return frc2::cmd::Parallel(cmd::AimAtPose(target),
    SubShooter::GetInstance().SetShooterTargetFromDist(distanceToTarget),
    SubHood::GetInstance().SetHoodPositionTargetFromDist(distanceToTarget))
    .Until([] {
      return SubShooter::GetInstance().IsAtSpeed() && SubTurret::GetInstance().TurretIsAtTarget() &&
             SubHood::GetInstance().HoodIsAtTarget();
    })
    .AndThen(frc2::cmd::Parallel(SubIntake::GetInstance().IntakeOn(),
      SubFeeder::GetInstance().FeederOn(), SubIndexer::GetInstance().IndexerOn()));
}

}  // namespace cmd