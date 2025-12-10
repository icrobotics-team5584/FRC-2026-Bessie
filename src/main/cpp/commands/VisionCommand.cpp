#include "commands/VisionCommand.h"
#include "subsystems/SubDrivebase.h"
#include "subsystems/SubVision.h"
#include "subsystems/PoseEstimator.h"

namespace cmd {
using namespace frc2::cmd;

frc2::CommandPtr AddVisionMeasurement() {
    return RunOnce([] {
        auto poses = SubVision::GetInstance().GetEstPose();
        for (auto pose : poses) {
            PoseEstimator::GetInstance().AddVisionMeasurement(
                    pose.pose, pose.timestamp, {pose.dev, pose.dev, 0.9});
        }
    },{&SubVision::GetInstance()}).IgnoringDisable(true);
}

}