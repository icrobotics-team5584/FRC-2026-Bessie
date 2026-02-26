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
        if (Logger::Tune("Vision/Add pose measurement", !frc::RobotBase::IsSimulation())) { return; }

        auto cameras = SubVision::GetInstance().GetCameras();

        // Record of closest pose estimation
        units::length::meter_t bestDistance = 100_m;
        std::string bestName;
        frc::Pose2d bestPose;
        units::time::second_t bestTimestamp;

        // Loop over results from each camera
        for (auto cam : cameras) {
            Logger::FieldDisplay::GetInstance().DisplayPose("Vision/" + cam->GetCamName() + "/Est pose" , {});
            
            std::string name = cam->GetCamName();

            if (!cam->CanSeeTag()) {
                Logger::Log("Vision/" + name + "/Can see tag" , false);
                continue;
            }
            Logger::Log("Vision/" + name + "/Can see tag" , true);
            
            if (!SubVision::GetInstance().IsEstimateUsable(cam->GetEstPose())) {
                Logger::Log("Vision/" + name + "/Estimation Usable", false);
                continue;
            }
            Logger::Log("Vision/"+ name +"/Estimation Usable", true);

            auto estPose = cam->GetEstPose().estimatedPose.ToPose2d();
            
            // Turret camera to robot center transformation
            if (name == SubVision::GetInstance().TURRET_CAM_NAME) {
                frc::Translation2d turret_to_cam = {SubVision::TURRET_TO_CAM.X(),SubVision::TURRET_TO_CAM.Y()};
                frc::Rotation2d turr_ang = SubTurret::GetInstance().GetTurretAngleAtTime(cam->GetEstPose().timestamp);

                frc::Translation2d r_turret_to_cam = turret_to_cam.RotateBy(turr_ang);
                frc::Transform2d t_turret_to_cam {r_turret_to_cam.X(), r_turret_to_cam.Y(), turr_ang};

                estPose = cam->GetEstPose().estimatedPose.ToPose2d()
                                        .TransformBy(t_turret_to_cam.Inverse())
                                        .TransformBy(SubTurret::ROBOT_TO_TURRET.Inverse());
            }

            // Compare for best
            auto distance = SubVision::GetInstance().GetAvgDistanceFromCamera(cam->GetEstPose());
            if (distance < bestDistance) {
                bestDistance = distance;
                bestName = name;
                bestPose = estPose;
                bestTimestamp = cam->GetEstPose().timestamp;
            }                    
            Logger::FieldDisplay::GetInstance().DisplayPose("Vision/"+name+"/Est pose", estPose);
        }

        // Input the best pose to PoseHandler to update pose estimation
        if (bestDistance < 100_m) {
            double d = SubVision::GetInstance().GetDev(bestDistance);
            wpi::array<double,3> dev = {d, d, 0.9};
            PoseHandler::GetInstance().AddVisionMeasurement(bestPose, bestTimestamp, dev);
            Logger::FieldDisplay::GetInstance().DisplayPose("Vision/Est pose", bestPose);
            Logger::Log("Vision/Best pose source", bestName);
        }

    }, {&SubVision::GetInstance()}
).IgnoringDisable(true);
}

}