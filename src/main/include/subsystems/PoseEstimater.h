// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc/smartdashboard/Field2d.h>
#include "DrivebaseConfig.h"

class PoseEstimater {
public:
  PoseEstimater();
  static PoseEstimater& GetInstance() {
    static PoseEstimater inst;
    return inst;
  }

  // Update pose
  void DrivebaseUpdate(frc::Rotation2d gyroAngle, wpi::array<frc::SwerveModulePosition, 4U> &wheelPositions);
  void AddVisionMeasurement(frc::Pose2d pose, units::second_t timeStamp, wpi::array<double,3> dev);

  // GetPose
  frc::Pose2d GetEstPose();

private:
  frc::SwerveDriveKinematics<4> _kinematics {
      DrivebaseConfig::FL_POSITION,
      DrivebaseConfig::FR_POSITION,
      DrivebaseConfig::BL_POSITION,
      DrivebaseConfig::BR_POSITION
  };
  frc::SwerveDrivePoseEstimator<4> _poseEstimator{
      _kinematics,
      0_deg,
      {frc::SwerveModulePosition{0_m, 0_deg},
       frc::SwerveModulePosition{0_m, 0_deg},
       frc::SwerveModulePosition{0_m, 0_deg},
       frc::SwerveModulePosition{0_m, 0_deg}},
      frc::Pose2d()};

  frc::Field2d _fieldDisplay;

  // Sim pose estimation
  frc::SwerveDrivePoseEstimator<4> _simPoseEstimator{
      _kinematics,
      0_deg,
      {frc::SwerveModulePosition{0_m, 0_deg},
       frc::SwerveModulePosition{0_m, 0_deg},
       frc::SwerveModulePosition{0_m, 0_deg},
       frc::SwerveModulePosition{0_m, 0_deg}},
      frc::Pose2d()};
};
