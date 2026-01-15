// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ShootCommands.h"
#include "subsystems/SubTurret.h"
#include "subsystems/SubHood.h"
#include "subsystems/SubShooter.h"
#include "subsystems/SubDrivebase.h"
#include "utilities/PoseHandler.h"
#include <frc2/command/Commands.h>

ShootConfig CalShootOnMove(double shooter_h, frc::Translation3d target, frc::Rotation2d piv,
                               units::meters_per_second_t bot_x, units::meters_per_second_t bot_y)
{
    frc::Pose2d orig = PoseHandler::GetInstance().GetPose();
    double distance = hypot(target.X().value() - orig.X().value(), target.Y().value() - orig.Y().value());
    double height = target.Z().value() - shooter_h;
    double piv_rad = piv.Radians().value();

    double time = 0.101937 * (
        2.21472 * sin(piv_rad) *
        sqrt(
            (distance*distance * (1 / cos(piv_rad))) / 
            (-height + distance * tan(piv_rad))
        ) +
        sqrt(
            -19.62 * height + 
            (4.905 * distance * distance * sin(piv_rad) * tan(piv_rad)) / 
            (-height * cos(piv_rad) + distance * sin(piv_rad))
        )
    );

    frc::Translation3d new_pos {orig.X() + bot_x * time * 1_s, orig.Y() + bot_y * time * 1_s, shooter_h * 1_m};

    double new_distance = hypot(target.X().value() - new_pos.X().value(), target.Y().value() - new_pos.Y().value());

    frc::Rotation2d new_yaw = atan2(target.Y().value() - new_pos.Y().value(), target.X().value() - new_pos.X().value()) * 1_rad;

    frc::Rotation2d new_piv_ang = atan((height + 4.905 * time * time) / new_distance) * 1_rad;

    units::meters_per_second_t new_vel = new_distance / cos(new_piv_ang.Radians().value()) / time * 1_mps;

    return ShootConfig{
        new_piv_ang,
        new_yaw,
        new_vel
    };
}

namespace cmd {
using namespace frc2::cmd;

frc2::CommandPtr AimAndShoot(frc::Translation3d target_pose) {
    return Run([target_pose] {
        auto curr_pos = PoseHandler::GetInstance().GetPose();
        auto vel = SubDrivebase::GetInstance().GetVelocity();
        auto yaw = SubDrivebase::GetInstance().GetGyroAngle().Radians().value();
        units::meter_t distance = hypot(target_pose.X().value() - curr_pos.X().value(), target_pose.Y().value() - curr_pos.Y().value()) * 1_m;
        ShootConfig conf = CalShootOnMove(0.5, target_pose, SubHood::GetInstance().GetAngleFromDistance(distance),
                                                                       vel * cos(yaw), vel * sin(yaw));
        SubHood::GetInstance().SetHoodPos(90_deg - conf.PivotAngle.Degrees());
        SubTurret::GetInstance().SetAngle(conf.Yaw.Degrees());
        SubShooter::GetInstance().SetTargetFromProjectileVel(conf.Velocity);
    })
    .FinallyDo([]{SubShooter::GetInstance().Stop();});
}

}