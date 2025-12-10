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