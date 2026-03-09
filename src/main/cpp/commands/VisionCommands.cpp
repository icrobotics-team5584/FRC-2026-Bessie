// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/VisionCommands.h"
#include "subsystems/Turret/SubTurret.h"
#include "subsystems/SubVision.h"
#include "utilities/Logger.h"
#include "utilities/PoseHandler.h"
#include <frc/RobotBase.h>

namespace cmd {
using namespace frc2::cmd;

frc2::CommandPtr AddVisionMeasurement() {
    return Run([] {
        if (Logger::Tune("Vision/Add pose measurement", frc::RobotBase::IsSimulation())) {return;}

        auto poses = SubVision::GetInstance().GetPose();

        std::vector<ProcessedPose> processedResults = {};

        for (auto [name,pose] : poses) {
            Logger::FieldDisplay::GetInstance().DisplayPose("Vision/"+name+"/Est pose" , {});

            // If the pose has value
            Logger::Log("Vision/"+name+"/Have value" , pose.has_value());
            if (!pose.has_value()) {continue;}

            // If the pose is useable, or the 
            bool poseUseable = SubVision::GetInstance().IsEstimateUsable(pose.value());
            bool timestampVaild = frc::Timer::GetFPGATimestamp() - pose.value().timestamp < 0.1_s;
            Logger::Log("Vision/"+name+"/Est pose useable", poseUseable);
            Logger::Log("Vision/"+name+"/Vaild timestamp", timestampVaild);
            if (!poseUseable || !timestampVaild) {continue;}
            
            // Calculate real robot pose if using turret camera
            frc::Pose2d botPose;
            if (name == SubVision::GetInstance().TURRET_CAM_NAME) {
                // Turret
                frc::Transform2d t_bot_to_turret = SubTurret::ROBOT_TO_TURRET;
                frc::Translation2d turret_to_cam = {SubVision::TURRET_TO_CAM.X(),SubVision::TURRET_TO_CAM.Y()};
                frc::Rotation2d turr_ang = SubTurret::GetInstance().GetTurretAngleAtTime(pose.value().timestamp);

                frc::Translation2d r_turret_to_cam = turret_to_cam.RotateBy(turr_ang);
                frc::Transform2d t_turret_to_cam {r_turret_to_cam.X(), r_turret_to_cam.Y(), turr_ang};

                frc::Pose2d botPose = pose.value().estimatedPose.ToPose2d()
                                            .TransformBy(t_turret_to_cam.Inverse())
                                            .TransformBy(t_bot_to_turret.Inverse());
            } else {
                // Static camera
                botPose = pose.value().estimatedPose.ToPose2d();
            }
            
            // Distance between tag and camera
            units::length::meter_t distance = SubVision::GetInstance().GetAvgDistanceFromCamera(pose.value());

            processedResults.push_back({name, botPose, pose.value().timestamp, distance});
        }

        // Compare results, prioritize static camera with cloest distance
        if (processedResults.empty()) {return;}
        
        ProcessedPose bestResult = processedResults.front();

        for (auto result : processedResults) {
            // If turret camera
            if (result.camName == SubVision::GetInstance().TURRET_CAM_NAME &&
                bestResult.camName != SubVision::GetInstance().TURRET_CAM_NAME) {
                continue;
            }

            // Compare distance
            if (result.distance < bestResult.distance) {
                bestResult = result;
            }
        }

        // Add result to PoseHandler
        double dev = SubVision::GetInstance().GetDev(bestResult.distance);
        PoseHandler::GetInstance().AddVisionMeasurement(bestResult.pose, bestResult.timestamp, {dev, dev, 0.9});

    }, {&SubVision::GetInstance()}
).IgnoringDisable(true);
}

}