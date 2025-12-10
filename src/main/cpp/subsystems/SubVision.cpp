// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubVision.h"
#include "subsystems/SubDrivebase.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DriverStation.h>
#include <frc/RobotBase.h>
#include <photon/estimation/CameraTargetRelation.h>
#include <frc/MathUtil.h>
#include "utilities/PoseHandler.h"
#include "utilities/Logger.h"


SubVision::SubVision() {
  // Append camera to camera list

  _camList.emplace_back(
    &_leftCamera,
    &_leftPoseEstimater,
    &_leftEstPose
  );

  _camList.emplace_back(
    &_rightCamera,
    &_rightPoseEstimater,
    &_rightEstPose
  );

   // Set dev table for distance based deviance
  _devTable.insert(0_m, 0);
  _devTable.insert(0.71_m, 0.002);
  _devTable.insert(1_m, 0.006);
  _devTable.insert(1.5_m, 0.02);
  _devTable.insert(2_m, 0.068);
  _devTable.insert(3_m, 0.230);

  _leftPoseEstimater.SetMultiTagFallbackStrategy(photon::PoseStrategy::LOWEST_AMBIGUITY);

  // Sim set up
  _visionSim.AddAprilTags(_tagMap);
  _visionSim.AddCamera(&_leftCamSim, _leftBotToCam);
  _visionSim.AddCamera(&_rightCamSim, _rightBotToCam);

  // Display tags on field
  for (auto target : _visionSim.GetVisionTargets()) {
     Logger::FieldDisplay::GetInstance().DisplayPose(fmt::format("tag{}", target.fiducialId),
                                            target.GetPose().ToPose2d());
  }

  // Call this once just to get rid of the warnings that it is unused.
  // Its a photonlib bug.
  photon::VisionEstimation::EstimateCamPosePNP({}, {}, {}, {}, photon::TargetModel{1_m});
}

void SubVision::Periodic() {
  UpdateVision();
  LogStatus();
}

void SubVision::LogStatus() {
  Logger::Log("Vision/Last saw tag/id", _lastTag.tag.fiducialId);
  Logger::Log("Vision/Last saw tag/timestamp", _lastTag.timestamp);
}

void SubVision::SimulationPeriodic() {
  _visionSim.Update(PoseHandler::GetInstance().GetSimPose());
}

void SubVision::UpdateVision() {
  double largestArea = 0;
  std::vector<std::string> targets[_camList.size()];
  int id = 0;

  for (auto cam : _camList) {
    std::vector<photon::PhotonPipelineResult> results = cam.camera->GetAllUnreadResults();
    auto resultCount = results.size();
    if (resultCount > 0) {
      for (auto result : results) {
        *cam.estPose = cam.poseEstimater->Update(result);
        for (const auto& target : result.targets) {
          double targetArea = target.GetArea();
          if (targetArea > largestArea ) {
            if(cam.estPose->has_value()) {
              _lastTag.timestamp = cam.estPose->value().timestamp;
              _lastTag.tag = target;
              _lastTag.cameraId = id;
            }

            largestArea = targetArea;
          }
        }
      }
    }
    id++;
  }
}

std::vector<PoseEstimate> SubVision::GetEstPose() {
  std::vector<PoseEstimate> l;
  int i = 0;
  for (auto cam : _camList) {
    if (cam.estPose->has_value()) {
      double d = GetDev(cam.estPose->value());
      l.push_back({cam.estPose->value().estimatedPose.ToPose2d(),d, cam.estPose->value().timestamp});
      i++;
    }
  }
  Logger::Log("Vision/Est pose number", i);
  return l;
}

int SubVision::GetLastSeenTagID() {
  return _lastTag.tag.GetFiducialId();
}

int SubVision::GetLastCameraUsed() {
  return _lastTag.cameraId;
}

double SubVision::GetDev(photon::EstimatedRobotPose pose) {
  units::meter_t distance = 0_m;
  if (pose.targetsUsed.size() == 0) {
    return 0;
  }
  for (auto target : pose.targetsUsed) {
    distance += target.GetBestCameraToTarget().Translation().Norm();
  }
  distance /= pose.targetsUsed.size();
  return _devTable[distance];
}

bool SubVision::IsEstimateUsable(photon::EstimatedRobotPose pose) {
  units::meter_t distance = 0_m;
  auto tagCount = pose.targetsUsed.size();
  bool hasMyTargets = false;
  if (pose.targetsUsed.size() == 0) {
    return 0;
  }
  for (auto target : pose.targetsUsed) {
    distance += target.GetBestCameraToTarget().Translation().Norm();
  }
  distance /= pose.targetsUsed.size();


  return ((distance < 0.7_m) || (tagCount > 1)) && hasMyTargets;
}

frc::Pose2d SubVision::CalculateRelativePose(frc::Pose2d pose, units::meter_t x, units::meter_t y) {
  frc::Translation2d trans {x,y};
  return frc::Pose2d{pose.Translation() + trans.RotateBy(pose.Rotation()), pose.Rotation()};
}

std::optional<frc::Pose2d> SubVision::GetAprilTagPose(int id) {
  auto pose = _tagMap.GetTagPose(id);
  if (pose.has_value()) {
    return pose.value().ToPose2d();
  } else {
    return std::nullopt;
  }
}

int SubVision::GetClosestTag(frc::Pose2d currentPose){
  int closestReef = 0;
  units::length::meter_t closestDistance;
  std::vector<frc::AprilTag> tagList = _tagMap.GetTags();

   for (const frc::AprilTag tag : tagList) {
    int id = tag.ID;
    auto distance = currentPose.Translation().Distance(GetAprilTagPose(id).value().Translation());
    if (closestReef == 0 || distance < closestDistance) {
      closestDistance = distance;
      closestReef = id;
    }
  }

  return closestReef;
}