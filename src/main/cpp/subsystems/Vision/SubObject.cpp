// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Vision/SubObject.h"

#include <vector>
#include <optional>

SubObject::SubObject() = default;

// This method will be called once per scheduler run
void SubObject::Periodic() {}


frc2::CommandPtr SubObject::UpdateLargestTarget() {
    std::optional<LimelightHelpers::RawDetection> GetLargestRawDetection(const std::string& limelightName) {
    
    // Retrieve a flat vector of raw neural detections directly from NetworkTables
    std::vector<LimelightHelpers::RawDetection> detections = LimelightHelpers::getRawDetections(limelightName);
    
    LimelightHelpers::RawDetection largestTarget;
    double maxArea = -1.0;
    bool targetFound = false;

    // Iterate through all currently visible targets
    for (const auto& detection : detections) {
        // ta represents the area of this individual target
        double currentArea = detection.ta; 

        if (currentArea > maxArea) {
            maxArea = currentArea;
            largestTarget = detection;
            targetFound = true;
        }
    }

    if (targetFound) {
        targId = largestTarget.targId;
        targTx = largestTarget.targTx;
        targTy = largestTarget.targTy;
        targTA = largestTarget.targTA;
        bool targetFound = true;
        return largestTarget;
    } else {
        bool targetFound = false;
        return std::nullopt; // Returns an empty optionl if nothing is seena
    }
}
}