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
  
  //sqrt[(curX-TargX)^2 + (curY-TargY)^2] <- pythag
  units::meter_t distanceToTarget = sqrt(pow((currentPose.X().value()-target.X().value()),2) + pow((currentPose.Y().value()-target.Y().value()),2))*1_m;

  return cmd::AimAtPose(target)
  .AlongWith(SubHood::GetInstance().SetHoodPositionTargetFromDist(distanceToTarget))
  .AlongWith(SubShooter::GetInstance().SetShooterTargetFromDist(distanceToTarget))
  .Until([]{return SubShooter::GetInstance().IsAtSpeed();})
  .AndThen(SubFeeder::GetInstance().FeederOn())
  .AlongWith(SubIntake::GetInstance().IntakeOn())
  .AlongWith(SubIndexer::GetInstance().IndexerOn());
}

}  // namespace cmd