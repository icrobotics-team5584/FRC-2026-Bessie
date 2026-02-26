// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "utilities/ICCamera.h"
#include "utilities/PoseHandler.h"
#include "utilities/Logger.h"
#include <frc/smartdashboard/SmartDashboard.h>

ICCamera::ICCamera(std::string name, frc::Transform3d botToCam, frc::AprilTagFieldLayout tagMap, units::second_t lastTime)
 : _camName(name), _botToCam(botToCam), _tagMap(tagMap), _cam(name),
 _camSim(&_cam), _poseEstimator(_tagMap, _botToCam), _lastTime(lastTime)
 {
    Logger::Log("Vision/" + _camName + "/Is Connected", _cam.IsConnected());
 }

void ICCamera::Update() {
    // Get results from camera
    auto results = _cam.GetAllUnreadResults();
    if (results.empty()) {
        return;
    }

    // Analyse the latest reading only
    _result = results.back();
    auto est = _poseEstimator.EstimateCoprocMultiTagPose(_result);
    if (!est.has_value()) {
        return;
    }
    _estPose = est.value();
    
    // Record seen tag to dashboard
    std::string targets = "";
    for (const photon::PhotonTrackedTarget& target : _result.targets) {
        targets += std::to_string(target.GetFiducialId()) + ", ";
    }
    frc::SmartDashboard::PutString("Vision/" + _camName + "/targets", targets);
}

std::string ICCamera::GetCamName() {
    return _camName;
}

frc::Transform3d ICCamera::GetBotToCam() {
    return _botToCam;
}

photon::PhotonCameraSim* ICCamera::GetCamSim() {
    return &_camSim;
}

photon::EstimatedRobotPose ICCamera::GetEstPose() {
    return _estPose;
}

bool ICCamera::CanSeeTag() {
    return frc::Timer::GetFPGATimestamp() - _estPose.timestamp < _lastTime;
}

std::optional<frc::Transform3d> ICCamera::CalculateRobotToCamera(
  photon::PhotonPipelineResult &result, frc::Transform3d robotToTag) {
  if (result.HasTargets()) {
    auto target = result.GetBestTarget();
    frc::Transform3d cameraToTag = target.GetBestCameraToTarget();
    frc::Transform3d tagToCamera = cameraToTag.Inverse();
    frc::Transform3d robotToCamera = robotToTag + tagToCamera;
    
    return robotToCamera;
  } else {
    return std::nullopt;
  }
}

void ICCamera::CalibrateRobotToCamera(frc::Transform3d robotToTag) {
    Logger::Log("Vision/RobotToCamera/"+_camName+"/Result received", CanSeeTag());

    if (!CanSeeTag()) { return; }

    auto calcResult = CalculateRobotToCamera(_result, robotToTag);

    if (calcResult.has_value()) {
        auto robotToCamera = calcResult.value();
        Logger::Log("Vision/RobotToCamera/"+_camName+"/X", robotToCamera.X());
        Logger::Log("Vision/RobotToCamera/"+_camName+"/Y", robotToCamera.Y());
        Logger::Log("Vision/RobotToCamera/"+_camName+"/Z", robotToCamera.Z());
        Logger::Log("Vision/RobotToCamera/"+_camName+"/~X_Roll", robotToCamera.Rotation().X().convert<units::degree>());
        Logger::Log("Vision/RobotToCamera/"+_camName+"/~Y_Pitch", robotToCamera.Rotation().Y().convert<units::degree>());
        Logger::Log("Vision/RobotToCamera/"+_camName+"/~Z_Yaw", robotToCamera.Rotation().Z().convert<units::degree>());

        auto estimatedLeftCamPose = frc::Pose3d{PoseHandler::GetInstance().GetPose()}.TransformBy(robotToCamera);
        Logger::FieldDisplay::GetInstance().DisplayPose("Estimated-"+_camName+"-Pose", estimatedLeftCamPose.ToPose2d());
    }
}