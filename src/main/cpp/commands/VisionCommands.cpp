#include "commands/VisionCommands.h"

#include "subsystems/SubDrivebase.h"
#include "subsystems/SubVision.h"

#include "utilities/Logger.h"
#include "utilities/PoseHandler.h"

namespace cmd {
using namespace frc2::cmd;

frc2::CommandPtr AddVisionMeasurement() {
  return Run([] {
    auto estimatePoses = SubVision::GetInstance().GetPose();

    auto leftPose = estimatePoses[SubVision::Left];
    if (leftPose.has_value()) {
      Logger::Log("Vision/Left/Estimate has value", true);
      if (SubVision::GetInstance().IsEstimateUsable(leftPose.value())) {
        Logger::Log("Vision/Left/Estimate is usable", true);

        auto estimatedPose = leftPose.value();
        double d = SubVision::GetInstance().GetDev(estimatedPose);
        wpi::array<double, 3> dev = {d, d, 0.9};
        PoseHandler::GetInstance().AddVisionMeasurement(
          estimatedPose.estimatedPose.ToPose2d(), estimatedPose.timestamp, dev);
        Logger::FieldDisplay::GetInstance().DisplayPose(
          "LastLeftEstimatedPose", estimatedPose.estimatedPose.ToPose2d());
      } else {
        Logger::Log("Vision/Left/Estimate is usable", false);

        Logger::FieldDisplay::GetInstance().DisplayPose(
          "LastDiscardedLeftEstimatedPose", leftPose.value().estimatedPose.ToPose2d());
      }
    } else {
      Logger::Log("Vision/Left/Estimate has value", false);
      Logger::Log("Vision/Left/Estimate is usable", false);
    }

    auto rightPose = estimatePoses[SubVision::Right];
    if (rightPose.has_value()) {
      Logger::Log("Vision/Right/Estimate has value", true);
      if (SubVision::GetInstance().IsEstimateUsable(rightPose.value())) {
        Logger::Log("Vision/Right/Estimate is usable", true);

        auto estimatedPose = rightPose.value();
        double d = SubVision::GetInstance().GetDev(estimatedPose);
        wpi::array<double, 3> dev = {d, d, 0.9};
        PoseHandler::GetInstance().AddVisionMeasurement(
          estimatedPose.estimatedPose.ToPose2d(), estimatedPose.timestamp, dev);
        Logger::FieldDisplay::GetInstance().DisplayPose(
          "LastRightEstimatedPose", estimatedPose.estimatedPose.ToPose2d());
      } else {
        Logger::Log("Vision/Right/Estimate is usable", false);

        Logger::FieldDisplay::GetInstance().DisplayPose(
          "LastDiscardedRightEstimatedPose", rightPose.value().estimatedPose.ToPose2d());
      }
    } else {
      Logger::Log("Vision/Right/Estimate has value", false);
      Logger::Log("Vision/Right/Estimate is usable", false);
    }
  },
  {&SubVision::GetInstance()})
  .IgnoringDisable(true);
}

}  // namespace cmd