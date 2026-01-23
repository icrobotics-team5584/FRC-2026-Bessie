#include "commands/FuelCommands.h"

#include "subsystems/SubDeploy.h"
#include "subsystems/SubIndexer.h"
#include "subsystems/SubIntake.h"
#include "commands/TurretCommands.h"
#include "subsystems/SubHood.h"
#include "subsystems/SubShooter.h"
#include "utilities/PoseHandler.h"
#include "subsystems/SubFeeder.h"

namespace cmd {
frc2::CommandPtr IntakeSequence() {
  return SubDeploy::GetInstance()
    .DeployIntake()
    .AlongWith(SubIntake::GetInstance().IntakeOn())
    .AlongWith(SubIndexer::GetInstance().IndexerOn());
}

frc2::CommandPtr StationaryShootAt(frc::Pose2d target){
  frc::Pose2d currentPose = PoseHandler::GetInstance().GetPose();
  
  units::meter_t distanceToTarget = target.Translation().Distance(currentPose.Translation());

  return cmd::AimAtPose(target)
  .AlongWith(SubHood::GetInstance().SetHoodPositionTargetFromDist(distanceToTarget))
  .AlongWith(SubShooter::GetInstance().SetShooterTargetFromDist(distanceToTarget))
  .Until([]{return SubShooter::GetInstance().IsAtSpeed();})
  .AndThen(SubFeeder::GetInstance().FeederOn())
  .AlongWith(SubIntake::GetInstance().IntakeOn())
  .AlongWith(SubIndexer::GetInstance().IndexerOn());
}

}  // namespace cmd