// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/PoseEstimator.h"
#include <frc/DriverStation.h>
#include "utilities/Logger.h"

PoseEstimator::PoseEstimator() {}

void PoseEstimator::DrivebaseUpdate(frc::Rotation2d gyroAngle, wpi::array<frc::SwerveModulePosition, 4U> &wheelPositions) {
    auto alliance = frc::DriverStation::GetAlliance();
    if (alliance.value_or(frc::DriverStation::Alliance::kBlue) ==
        frc::DriverStation::Alliance::kBlue) {
        _poseEstimator.Update(gyroAngle, wheelPositions);
    } else {
        _poseEstimator.Update(gyroAngle - 180_deg, wheelPositions);
    }

    _fieldDisplay.SetRobotPose(_poseEstimator.GetEstimatedPosition());
}

void PoseEstimator::AddVisionMeasurement(frc::Pose2d pose, units::second_t timeStamp, wpi::array<double,3> dev) {
    Logger::Log("Vision/Updated pose", 1);
    Logger::Log("Vision/Est pose/x",pose.X());
    Logger::Log("Vision/Est pose/y",pose.Y());
    _poseEstimator.AddVisionMeasurement(pose, timeStamp, dev);
    Logger::Log("Vision/Curr pose/x",_poseEstimator.GetEstimatedPosition().X());
    Logger::Log("Vision/Curr pose/y",_poseEstimator.GetEstimatedPosition().Y());
    _fieldDisplay.SetRobotPose(_poseEstimator.GetEstimatedPosition());
}

frc::Pose2d PoseEstimator::GetEstPose() {
    return _poseEstimator.GetEstimatedPosition();
}

ShootConfig PoseEstimator::CalShootOnMove(double shooter_h, frc::Translation3d target, frc::Rotation2d piv,
                               units::meters_per_second_t bot_x, units::meters_per_second_t bot_y)
{
    frc::Pose2d orig = GetEstPose();
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