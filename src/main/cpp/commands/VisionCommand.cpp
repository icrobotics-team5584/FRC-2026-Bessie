#include "commands/VisionCommand.h"
#include "subsystems/SubDrivebase.h"
#include "subsystems/SubVision.h"
#include "subsystems/PoseEstimater.h"

namespace cmd {
using namespace frc2::cmd;

frc2::CommandPtr AddVisionMeasurement() {
    auto poses = SubVision::GetInstance().GetEstPose();
    for (auto pose : poses) {
        PoseEstimater::GetInstance().AddVisionMeasurement(
                pose.pose, pose.timestamp, {pose.dev, pose.dev, 0.9});
    }
}

}