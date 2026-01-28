// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "utilities/Camera.h"
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

    std::vector<photon::PhotonPipelineResult> results = _cam.GetAllUnreadResults();
    if (results.size() > 0) {
        for (photon::PhotonPipelineResult result : results) {
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

Camera::TagObservation Camera::GetLastTag() {
    return _lastTagObservation;
}

std::optional<photon::EstimatedRobotPose> Camera::GetEstPose() {
    return _estPose;
}