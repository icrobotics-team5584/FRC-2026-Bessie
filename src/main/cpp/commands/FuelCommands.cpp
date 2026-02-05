#include "commands/FuelCommands.h"

#include "subsystems/SubDeploy.h"
#include "subsystems/SubFeeder.h"
#include "subsystems/SubHood.h"
#include "subsystems/SubIndexer.h"
#include "subsystems/SubIntake.h"
#include "subsystems/SubShooter.h"
#include "subsystems/SubTurret.h"
#include "utilities/Logger.h"
#include "utilities/ShotPlanner.h"

#include "commands/TurretCommands.h"

#include "utilities/PoseHandler.h"
#include <frc/geometry/Transform2d.h>

namespace cmd {
frc2::CommandPtr IntakeSequence() {
  return SubDeploy::GetInstance()
    .DeployIntake()
    .AlongWith(SubIntake::GetInstance().IntakeOn());
}

frc2::CommandPtr StationaryShootAt(frc::Translation2d target) {
  auto distanceToTarget = [target] {
    auto curPose = PoseHandler::GetInstance().GetPose();
    auto turretPose = curPose.TransformBy(SubTurret::ROBOT_TO_TURRET);

    Logger::FieldDisplay::GetInstance().DisplayPose("Turret/turretPose", turretPose);
    Logger::Log("Shooter/distToTargetInner", target.Distance(turretPose.Translation()));

    return target.Distance(turretPose.Translation());};

  return frc2::cmd::Parallel(cmd::AimAtSpot(target),
    SubShooter::GetInstance().SpinWithDistance(distanceToTarget),
    SubHood::GetInstance().SetHoodPositionTargetFromDist(distanceToTarget))
    .Until([] {
      return SubShooter::GetInstance().IsAtSpeed() && SubTurret::GetInstance().IsAtTarget() &&
             SubHood::GetInstance().HoodIsAtTarget();
    })
    .AndThen(frc2::cmd::Parallel(SubIntake::GetInstance().IntakeOn(),
      SubFeeder::GetInstance().FeederOn(), SubIndexer::GetInstance().Index()));
}

frc2::CommandPtr ShootWhenReady() {
  return frc2::cmd::Either(SubFeeder::GetInstance().FeederOn().AlongWith(SubIndexer::GetInstance().Index()),
    SubFeeder::GetInstance().FeederOff().AlongWith(SubIndexer::GetInstance().StopIndex()),
    [] {
       auto currentPose = PoseHandler::GetInstance().GetPose();
       if (SubHood::GetInstance().HoodIsAtTarget() && SubShooter::GetInstance().IsAtSpeed() && SubTurret::GetInstance().IsAtTarget()){
              if (ShotPlanner::CalculateShotTarget(currentPose).shouldShoot){
              return true;
             }
       else {return false;}};
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

}  // namespace cmd