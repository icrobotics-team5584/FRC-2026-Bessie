// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/VisionCommands.h"
#include "subsystems/SubTurret.h"
#include "subsystems/SubVision.h"
#include "utilities/Logger.h"
#include "utilities/PoseHandler.h"
#include <frc/RobotBase.h>

void StableCameraProcess(std::string name, photon::EstimatedRobotPose pose) {
    double d = SubVision::GetInstance().GetDev(pose);
    wpi::array<double,3> dev = {d, d, 0.9};
    PoseHandler::GetInstance().AddVisionMeasurement(
        pose.estimatedPose.ToPose2d(), pose.timestamp, dev);
    Logger::FieldDisplay::GetInstance().DisplayPose("Vision/"+name+"/Est pose",
        pose.estimatedPose.ToPose2d());
}

void TurretCameraProcess(std::string name, photon::EstimatedRobotPose pose) {
    frc::Transform2d t_bot_to_turret = SubTurret::ROBOT_TO_TURRET;

    frc::Translation2d turret_to_cam = {SubVision::TURRET_TO_CAM.X(),SubVision::TURRET_TO_CAM.Y()};
    frc::Rotation2d turr_ang = SubTurret::GetInstance().GetTurretAngleAtTime(pose.timestamp);

    Logger::Log("Vision/turret/turr angle", turr_ang.Degrees());

    frc::Translation2d r_turret_to_cam = turret_to_cam.RotateBy(turr_ang);
    frc::Transform2d t_turret_to_cam {r_turret_to_cam.X(), r_turret_to_cam.Y(), turr_ang};

    frc::Pose2d orig_bot_pose = pose.estimatedPose.ToPose2d()
                                .TransformBy(t_turret_to_cam.Inverse())
                                .TransformBy(t_bot_to_turret.Inverse());

    double d = SubVision::GetInstance().GetDev(pose);
    wpi::array<double,3> dev = {d, d, 0.9};
    PoseHandler::GetInstance().AddVisionMeasurement(
        orig_bot_pose, pose.timestamp, dev);
    Logger::FieldDisplay::GetInstance().DisplayPose("Vision/"+name+"/Est pose",orig_bot_pose);
}

namespace cmd {
using namespace frc2::cmd;

frc2::CommandPtr AddVisionMeasurement() {
    return Run([] {
        if (Logger::Tune("Vision/Add pose measurement", !frc::RobotBase::IsSimulation())){
            auto poses = SubVision::GetInstance().GetPose();
            for (auto [name,pose] : poses) {
                Logger::FieldDisplay::GetInstance().DisplayPose("Vision/"+name+"/Est pose" , {});
                Logger::FieldDisplay::GetInstance().DisplayPose("Vision/"+name+"/Discarded est pose" , {});
                Logger::Log("Vision/"+name+"/Have value" , false);
                
                if (pose.has_value()) {
                    Logger::Log("Vision/"+name+"/Have value" , true);
                    auto estPose = pose.value();
                    if (SubVision::GetInstance().IsEstimateUsable(estPose)) {
                        if (name == "Turret") {
                            TurretCameraProcess(name, estPose);
                        } else {
                            StableCameraProcess(name, estPose);
                        }
                    } else {
                        Logger::FieldDisplay::GetInstance().DisplayPose("Vision/"+name+"/Discarded est pose",
                            {estPose.estimatedPose.ToPose2d()});
                    }
                }
            }
        }
    }, {&SubVision::GetInstance()}
).IgnoringDisable(true);
}

}