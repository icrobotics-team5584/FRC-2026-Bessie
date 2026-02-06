// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "utilities/ICCamera.h"
#include "utilities/PoseHandler.h"
#include "utilities/Logger.h"
#include <frc/smartdashboard/SmartDashboard.h>

ICCamera::ICCamera(std::string name, frc::Transform3d botToCam, frc::AprilTagFieldLayout tagMap, std::string label)
 : _camName(name), _botToCam(botToCam), _tagMap(tagMap), _cam(name),
 _camSim(&_cam), _poseEstimator(_tagMap, _botToCam)
 {
    if (label == "") {
        _label = _camName;
    } else {
        _label = label;
    }

    Logger::Log("Vision/" + _label + "/Is Connected", _cam.IsConnected());
 }

std::optional<photon::EstimatedRobotPose> ICCamera::Update() {
    double largestArea = 0;
    std::string targets = "";
    _estPose.reset();

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

std::string ICCamera::GetCamName() {
    return _camName;
}

std::string ICCamera::GetCamLabel() {
    return _label;
}

frc::Transform3d ICCamera::GetBotToCam() {
    return _botToCam;
}

photon::PhotonCameraSim* ICCamera::GetCamSim() {
    return &_camSim;
}

std::vector<photon::PhotonPipelineResult> ICCamera::GetLatestResult() {
    return _results;
}

ICCamera::TagObservation ICCamera::GetLastTagObservation() {
    return _lastTagObservation;
}

std::optional<photon::EstimatedRobotPose> ICCamera::GetEstPose() {
    return _estPose;
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