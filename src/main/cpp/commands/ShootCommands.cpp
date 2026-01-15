// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ShootCommands.h"
#include "subsystems/SubTurret.h"
#include "subsystems/SubHood.h"
#include "subsystems/SubShooter.h"
#include "subsystems/SubDrivebase.h"
#include "subsystems/PoseEstimator.h"
#include <frc2/command/Commands.h>

namespace cmd {
using namespace frc2::cmd;

frc2::CommandPtr AimAndShoot(frc::Translation3d target_pose) {
    return Run([target_pose] {
        auto curr_pos = PoseEstimator::GetInstance().GetEstPose();
        auto vel = SubDrivebase::GetInstance().GetVelocity();
        auto yaw = SubDrivebase::GetInstance().GetGyroAngle().Radians().value();
        units::meter_t distance = hypot(target_pose.X().value() - curr_pos.X().value(), target_pose.Y().value() - curr_pos.Y().value()) * 1_m;
        ShootConfig conf = PoseEstimator::GetInstance().CalShootOnMove(0.5, target_pose, SubHood::GetInstance().GetAngleFromDistance(distance),
                                                                       vel * cos(yaw), vel * sin(yaw));
        
        SubHood::GetInstance().SetHoodPosition(90_deg - conf.PivotAngle.Degrees());
        SubTurret::GetInstance().SetTurretAngle(conf.Yaw.Degrees());
        SubShooter::GetInstance().SetTargetFromProjectileVel(conf.Velocity);
    }).FinallyDo([] {
        SubShooter::GetInstance().StopShooter();
    });
}

}