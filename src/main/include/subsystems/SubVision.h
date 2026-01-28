
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Transform3d.h>
#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/apriltag/AprilTagFields.h>
#include <photon/PhotonCamera.h>
#include <photon/simulation/VisionSystemSim.h>
#include <photon/PhotonPoseEstimator.h>
#include <frc/Filesystem.h>
#include <wpi/interpolating_map.h>
#include "utilities/Camera.h"

class SubVision : public frc2::SubsystemBase {
public:
  SubVision();
  static SubVision& GetInstance() {
    static SubVision inst;
    return inst;
  }

  void Periodic() override;

  void SimulationPeriodic() override;

  enum Side {
    Left = 1,
    Right = 2
  };

  // Side GetLastCameraUsed();

  std::optional<frc::Pose2d> GetAprilTagPose(int id);

  std::map<std::string, std::optional<photon::EstimatedRobotPose>> GetPose();

  int GetClosestTag(frc::Pose2d currentPose);

  bool IsEstimateUsable(photon::EstimatedRobotPose pose);

  // int GetLastSeenTagID();

  double GetDev(photon::EstimatedRobotPose pose);

 private:

  struct TagObservation {
    photon::PhotonTrackedTarget tag;
    Side cameraSide;
    units::time::second_t timestamp;
  };
  struct TagObservation _lastTagObservation;

  //Create field layout
  std::string _tagMapFilePath = frc::filesystem::GetDeployDirectory() + "/2026-rebuilt.json";
  frc::AprilTagFieldLayout _tagMap{_tagMapFilePath};

  //Left camera config
  std::string _leftCamName = "ICR_OV9281_L";

  frc::Transform3d _leftBotToCam{{-350_mm,-470_mm,350_mm},{0_deg,-16_deg,190.54_deg}};

  Camera _leftCam {
    _leftCamName,
    _leftBotToCam,
    _tagMap,
    "left"
  };

  //Right camera config
  std::string _rightCamName = "ICR_OV9281_R";

  frc::Transform3d _rightBotToCam{{-350_mm,470_mm,350_mm},{0_deg,-16_deg,-190.54_deg}};

  Camera _rightCam {
    _rightCamName,
    _rightBotToCam,
    _tagMap,
    "right"
  };

  std::vector<Camera*> _camList {
    &_leftCam,
    &_rightCam
  };

  photon::VisionSystemSim _visionSim{"VisionSim"};

  //Deviation table for further distances from tag
  wpi::interpolating_map<units::meter_t, double> _devTable;
};