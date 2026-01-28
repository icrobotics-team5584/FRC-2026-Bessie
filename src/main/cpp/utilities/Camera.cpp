// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "utilities/Camera.h"
#include "utilities/PoseHandler.h"
#include "utilities/Logger.h"
#include <frc/smartdashboard/SmartDashboard.h>

Camera::Camera(std::string name, frc::Transform3d botToCam, frc::AprilTagFieldLayout tagMap, std::string label)
 : _camName(name), _botToCam(botToCam), _tagMap(tagMap), _cam(name),
 _camSim(&_cam), _poseEstimator(_tagMap, _botToCam)
 {
    if (label == "") {
        _label = _camName;
    } else {
        _label = label;
    }
 }

std::optional<photon::EstimatedRobotPose> Camera::Update() {
    double largestArea = 0;
    std::string targets = "";

     _results = _cam.GetAllUnreadResults();
    if (_results.size() > 0) {
        for (photon::PhotonPipelineResult result : _results) {
            _estPose = _poseEstimator.EstimateCoprocMultiTagPose(result);
            for (const photon::PhotonTrackedTarget& target : result.targets) {
                targets += std::to_string(target.GetFiducialId()) + ", ";
                double targetArea = target.GetArea();
                if (targetArea > largestArea) {
                    if (_estPose.has_value()) {
                        _lastTagObservation.timestamp = _estPose.value().timestamp;
                        _lastTagObservation.tag = target;
                    }
                    largestArea = targetArea;
                }
            }
        }
    }

    frc::SmartDashboard::PutString("Vision/" + _label + "/targets", targets);

    return _estPose;
}

std::string Camera::GetCamName() {
    return _camName;
}

std::string Camera::GetCamLabel() {
    return _label;
}

frc::Transform3d Camera::GetBotToCam() {
    return _botToCam;
}

photon::PhotonCameraSim* Camera::GetCamSim() {
    return &_camSim;
}

std::vector<photon::PhotonPipelineResult> Camera::GetLatestReading() {
    return _results;
}

Camera::TagObservation Camera::GetLastTag() {
    return _lastTagObservation;
}

std::optional<photon::EstimatedRobotPose> Camera::GetEstPose() {
    return _estPose;
}

std::optional<frc::Transform3d> Camera::CalculateRobotToCamera(
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

void Camera::CalibrateRobotToCamera(frc::Transform3d robotToTag) {
    //Left camera
    if (!_results.empty()) {
        auto calcResult = CalculateRobotToCamera(_results.back(), robotToTag);
        Logger::Log("Vision/RobotToCamera/"+_label+"/Result received", calcResult.has_value());

        if (calcResult.has_value()) {
            auto robotToCamera = calcResult.value();
            Logger::Log("Vision/RobotToCamera/"+_label+"/X", robotToCamera.X());
            Logger::Log("Vision/RobotToCamera/"+_label+"/Y", robotToCamera.Y());
            Logger::Log("Vision/RobotToCamera/"+_label+"/Z", robotToCamera.Z());
            Logger::Log("Vision/RobotToCamera/"+_label+"/~X_Roll", robotToCamera.Rotation().X().convert<units::degree>());
            Logger::Log("Vision/RobotToCamera/"+_label+"/~Y_Pitch", robotToCamera.Rotation().Y().convert<units::degree>());
            Logger::Log("Vision/RobotToCamera/"+_label+"/~Z_Yaw", robotToCamera.Rotation().Z().convert<units::degree>());

            auto estimatedLeftCamPose = frc::Pose3d{PoseHandler::GetInstance().GetPose()}.TransformBy(robotToCamera);
            Logger::FieldDisplay::GetInstance().DisplayPose("Estimated-"+_label+"-Pose", estimatedLeftCamPose.ToPose2d());
        }
    } else {
        Logger::Log("Vision/RobotToCamera/"+_label+"/Result received", false);
    }
}