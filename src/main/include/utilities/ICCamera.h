// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/geometry/Translation3d.h>
#include <frc2/command/Commands.h>
#include <photon/PhotonCamera.h>
#include <photon/PhotonPoseEstimator.h>
#include <photon/simulation/PhotonCameraSim.h>

#pragma once

class ICCamera {
public:
  ICCamera(std::string name, frc::Transform3d botToCam, frc::AprilTagFieldLayout tagMap, units::second_t lastTime = 0.08_s);

  void Update();

  struct TagObservation {
    photon::PhotonTrackedTarget tag;
    units::time::second_t timestamp;
  };

  std::string GetCamName();
  frc::Transform3d GetBotToCam();
  photon::PhotonCameraSim* GetCamSim();

  // The return object also include timestamp
  photon::EstimatedRobotPose GetEstPose();

  bool CanSeeTag();

  std::optional<frc::Transform3d> CalculateRobotToCamera(photon::PhotonPipelineResult &result, frc::Transform3d robotToTag);

  void CalibrateRobotToCamera(frc::Transform3d robotToTag);

private:
  std::string _camName;

  frc::Transform3d _botToCam;

  frc::AprilTagFieldLayout _tagMap;
  photon::PhotonCamera _cam;
  photon::PhotonCameraSim _camSim;
  photon::PhotonPoseEstimator _poseEstimator;

  units::second_t _lastTime;

  photon::PhotonPipelineResult _result;
  photon::EstimatedRobotPose _estPose { // Default pose
    { {0_m, 0_m, 0_m}, {0_deg, 0_deg, 0_deg} },
    -1_s,
    {},
    photon::PoseStrategy::MULTI_TAG_PNP_ON_COPROCESSOR
  };

  TagObservation _lastTagObservation;
};
