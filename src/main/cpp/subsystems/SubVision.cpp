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

   // Set dev table for distance based deviance
  _devTable.insert(0_m, 0);
  _devTable.insert(0.71_m, 0.002);
  _devTable.insert(1_m, 0.006);
  _devTable.insert(1.5_m, 0.02);
  _devTable.insert(2_m, 0.068);
  _devTable.insert(3_m, 0.230);

  // Sim set up
  _visionSim.AddAprilTags(_tagMap);
  _visionSim.AddCamera(&_leftCamSim, _leftBotToCam);
  _visionSim.AddCamera(&_rightCamSim, _rightBotToCam);

  // Display tags on field
  for (auto target : _visionSim.GetVisionTargets()) {
     Logger::FieldDisplay::GetInstance().DisplayPose(fmt::format("tag{}", target.fiducialId),
                                            target.GetPose().ToPose2d());
  }
}

void SubVision::Periodic() {
  frc::SmartDashboard::PutNumber("Vision/LastSeenTag", _lastTagObservation.tag.GetFiducialId());
  if (_lastTagObservation.cameraSide == Side::Left) {
    frc::SmartDashboard::PutString("Vision/Last Used Camera", "Left");
  } else {
    frc::SmartDashboard::PutString("Vision/Last Used Camera", "Right");
  }
  UpdateVision();
}

void SubVision::SimulationPeriodic() {
  _visionSim.Update(PoseHandler::GetInstance().GetSimPose());
}

void SubVision::UpdateVision() {
  double largestArea = 0;
  std::string leftTargets = "";
  std::string rightTargets = "";

  // Left camera
  std::vector<photon::PhotonPipelineResult> results = _leftCamera.GetAllUnreadResults();
  auto resultCount = results.size();
  if (resultCount > 0) {
    for (auto result : results) {
      _leftEstPose = _leftPoseEstimater.EstimateLowestAmbiguityPose(result);
      for (const auto& target : result.targets) {
        leftTargets += std::to_string(target.GetFiducialId()) + ", ";
        double targetArea = target.GetArea();
        if (targetArea > largestArea ) {

          if(_leftEstPose.has_value()) {
            _lastTagObservation.timestamp = _leftEstPose.value().timestamp;
            _lastTagObservation.tag = target;
            _lastTagObservation.cameraSide = Side::Left;
          }

          largestArea = targetArea;
        }
      }
    }
  }
  // Right camera
  results = _rightCamera.GetAllUnreadResults();
  resultCount = results.size();
  if (resultCount > 0) {
    for (auto result : results) {
      _rightEstPose = _rightPoseEstimater.EstimateLowestAmbiguityPose(result);

      for (const auto& target : result.targets) {
        rightTargets += std::to_string(target.GetFiducialId()) + ", ";
        double targetArea = target.GetArea();
        if (targetArea > largestArea) {
          if(_rightEstPose.has_value()) {
            _lastTagObservation.tag = target;
            _lastTagObservation.cameraSide = Side::Right;
            _lastTagObservation.timestamp = _rightEstPose.value().timestamp;          
          }
          largestArea = targetArea;
        }
      }
    }
  }

  frc::SmartDashboard::PutString("Vision/Left/targets", leftTargets);
  frc::SmartDashboard::PutString("Vision/Right/targets", rightTargets);
}

std::map<SubVision::Side, std::optional<photon::EstimatedRobotPose>> SubVision::GetPose() {
  return {{Left, _leftEstPose}, {Right, _rightEstPose}};
}

int SubVision::GetLastSeenTagID() {
  return _lastTagObservation.tag.GetFiducialId();
}

SubVision::Side SubVision::GetLastCameraUsed() {
  return _lastTagObservation.cameraSide;
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
  if (pose.targetsUsed.size() == 0) {
    return 0;
  }
  for (auto target : pose.targetsUsed) {
    distance += target.GetBestCameraToTarget().Translation().Norm();
  }
  distance /= pose.targetsUsed.size();

  return ((distance < 5_m) || (tagCount > 1));
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
  int closestTagID = 0;
  units::length::meter_t closestDistance;
  std::vector<frc::AprilTag> tagList = _tagMap.GetTags();

   for (const frc::AprilTag tag : tagList) {
    int id = tag.ID;
    auto distance = currentPose.Translation().Distance(GetAprilTagPose(id).value().Translation());
    if (closestTagID == 0 || distance < closestDistance) {
      closestDistance = distance;
      closestTagID = id;
    }
  }

  return closestTagID;
}

std::optional<frc::Transform3d> SubVision::CalculateRobotToCamera(
  photon::PhotonCamera &camera, frc::Transform3d robotToTag) {
  auto results = camera.GetAllUnreadResults();
  if (!results.empty()) {
    auto result = results.back();
    if (result.HasTargets()) {
      auto target = result.GetBestTarget();
      frc::Transform3d tagToCamera = target.GetBestCameraToTarget().Inverse();
      frc::Transform3d robotToCamera = robotToTag + tagToCamera;
      return robotToCamera;
    } else {
      return std::nullopt;
    }
  } else {
    return std::nullopt;
  }
}

frc2::CommandPtr SubVision::CalibrateRobotToCamera(frc::Translation3d robotToTag) {
  return frc2::cmd::RunOnce([this, robotToTag] {
    //Left camera
    auto leftResult = CalculateRobotToCamera(_leftCamera, frc::Transform3d{robotToTag, {}});
    Logger::Log("Vision/RobotToCamera/Left/Result Received", leftResult.has_value());
    if (leftResult.has_value()) {
      auto leftRobotToCamera = leftResult.value();
      Logger::Log("Vision/RobotToCamera/Left/X", leftRobotToCamera.X());
      Logger::Log("Vision/RobotToCamera/Left/Y", leftRobotToCamera.Y());
      Logger::Log("Vision/RobotToCamera/Left/Z", leftRobotToCamera.Z());
      Logger::Log("Vision/RobotToCamera/Left/Roll", leftRobotToCamera.Rotation().X());
      Logger::Log("Vision/RobotToCamera/Left/Pitch", leftRobotToCamera.Rotation().Y());
      Logger::Log("Vision/RobotToCamera/Left/Yaw", leftRobotToCamera.Rotation().Z());
    }

    //Right camera
    auto rightResult = CalculateRobotToCamera(_rightCamera, frc::Transform3d{robotToTag, {}});
    Logger::Log("Vision/RobotToCamera/Right/Result Received", rightResult.has_value());
    if (rightResult.has_value()) {
      auto rightRobotToCamera = rightResult.value();
      Logger::Log("Vision/RobotToCamera/Right/X", rightRobotToCamera.X());
      Logger::Log("Vision/RobotToCamera/Right/Y", rightRobotToCamera.Y());
      Logger::Log("Vision/RobotToCamera/Right/Z", rightRobotToCamera.Z());
      Logger::Log("Vision/RobotToCamera/Right/Roll", rightRobotToCamera.Rotation().X());
      Logger::Log("Vision/RobotToCamera/Right/Pitch", rightRobotToCamera.Rotation().Y());
      Logger::Log("Vision/RobotToCamera/Right/Yaw", rightRobotToCamera.Rotation().Z());
    }
  });
}