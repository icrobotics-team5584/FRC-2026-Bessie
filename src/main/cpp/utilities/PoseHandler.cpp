#include "utilities/PoseHandler.h"
#include "utilities/Logger.h"

PoseHandler::PoseHandler() {
}

frc::Pose2d PoseHandler::GetPose() {
    return _poseEstimator.GetEstimatedPosition();
}

frc::Pose2d PoseHandler::GetSimPose() {
    return _simPoseEstimator.GetEstimatedPosition();
}

void PoseHandler::Update(frc::Rotation2d angle, wpi::array<frc::SwerveModulePosition, 4U> states) {
    _poseEstimator.Update(angle, states);
    Logger::FieldDisplay::GetInstance().SetRobotPose(_poseEstimator.GetEstimatedPosition());
}

void PoseHandler::UpdateSim(frc::Rotation2d angle, wpi::array<frc::SwerveModulePosition, 4U> states, bool resetHeading, frc::Rotation2d heading) {
    _simPoseEstimator.Update(angle, states);
    if (resetHeading) { _simPoseEstimator.ResetRotation(heading); }
    Logger::FieldDisplay::GetInstance().DisplayPose("Sim pose", _simPoseEstimator.GetEstimatedPosition());
}

void PoseHandler::AddVisionMeasurement(frc::Pose2d pose, units::second_t timeStamp, wpi::array<double,3> dev) {
    Logger::Log("Vision/Updated pose", 1);
    Logger::Log("Vision/Est pose/x",pose.X());
    Logger::Log("Vision/Est pose/y",pose.Y());
    _poseEstimator.AddVisionMeasurement(pose, timeStamp, dev);
    Logger::Log("Vision/Curr pose/x",_poseEstimator.GetEstimatedPosition().X());
    Logger::Log("Vision/Curr pose/y",_poseEstimator.GetEstimatedPosition().Y());
    Logger::FieldDisplay::GetInstance().SetRobotPose(_poseEstimator.GetEstimatedPosition());
}