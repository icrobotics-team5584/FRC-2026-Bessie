
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

  /**
   * Update pose estimater with vision, should be called every frame
   */
  void UpdateVision();

  Side GetLastCameraUsed();

  std::optional<frc::Pose2d> GetAprilTagPose(int id);

  std::map<Side, std::optional<photon::EstimatedRobotPose>> GetPose();

  frc::Pose2d CalculateRelativePose(frc::Pose2d pose, units::meter_t xTransform, units::meter_t yTransform);

  int GetClosestTag(frc::Pose2d currentPose);

  bool IsEstimateUsable(photon::EstimatedRobotPose pose);

  int GetLastSeenTagID();

  double GetDev(photon::EstimatedRobotPose pose);

 private:

  struct TagObservation {
    photon::PhotonTrackedTarget tag;
    Side cameraSide;
    units::time::second_t timestamp;
  };
  struct TagObservation _lastTagObservation;

  //Create field layout
  std::string _tagMapFilePath = frc::filesystem::GetDeployDirectory() + "/-reefscape.json";
  frc::AprilTagFieldLayout _tagMap{_tagMapFilePath};

  //side configs
  std::array<int, 6> blueSide = {
    18, 19, 17, 21, 20, 22
  };

  std::array<int, 6> redSide = {
    7, 8, 6, 10, 9, 11
  };

  //Left camera config
  std::string _leftCamName = "ICR_OV2981_L (1)";

  photon::PhotonCamera _leftCamera{_leftCamName};

  photon::PhotonCameraSim _leftCamSim{&_leftCamera};
  photon::VisionSystemSim _visionSim{_leftCamName};

  frc::Transform3d _leftBotToCam{{-270_mm,270_mm,220_mm},{0_deg,5_deg,45_deg}};

  photon::PhotonPoseEstimator _leftPoseEstimater{
    _tagMap,
    photon::PoseStrategy::MULTI_TAG_PNP_ON_COPROCESSOR,
    _leftBotToCam
  };

  std::optional<photon::EstimatedRobotPose> _leftEstPose;

  //Right camera config
  std::string _rightCamName = "ICR_OV9281_R (1)";

  photon::PhotonCamera _rightCamera{_rightCamName};

  photon::PhotonCameraSim _rightCamSim{&_rightCamera};

  frc::Transform3d _rightBotToCam{{270_mm,270_mm,220_mm},{0_deg,5_deg,135_deg}};

  photon::PhotonPoseEstimator _rightPoseEstimater{
    _tagMap,
    photon::PoseStrategy::MULTI_TAG_PNP_ON_COPROCESSOR,
    _rightBotToCam
  };

  std::optional<photon::EstimatedRobotPose> _rightEstPose;

  //Deviation table for further distances from tag
  wpi::interpolating_map<units::meter_t, double> _devTable;
};
