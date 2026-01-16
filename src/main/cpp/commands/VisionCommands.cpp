#include "commands/VisionCommands.h"
#include "subsystems/SubDrivebase.h"
#include "subsystems/SubVision.h"
#include "utilities/PoseHandler.h"
#include "utilities/Logger.h"

namespace cmd {
using namespace frc2::cmd;

frc2::CommandPtr AddVisionMeasurement() {
return Run(
      [] {
        auto estimatePoses = SubVision::GetInstance().GetPose();

        auto leftPose = estimatePoses[SubVision::Left];
        if (leftPose.has_value()) {
          if (SubVision::GetInstance().IsEstimateUsable(leftPose.value())) {
            auto estimatedPose = leftPose.value();
            double d = SubVision::GetInstance().GetDev(estimatedPose);
            wpi::array<double,3> dev = {d, d, 0.9};
            PoseHandler::GetInstance().AddVisionMeasurement(
                estimatedPose.estimatedPose.ToPose2d(), estimatedPose.timestamp, dev);
            Logger::FieldDisplay::GetInstance().DisplayPose("LeftEstimatedPose",
                                                    estimatedPose.estimatedPose.ToPose2d());
          } else {
            Logger::FieldDisplay::GetInstance().DisplayPose("DiscardedLeftEstimatedPose", {leftPose.value().estimatedPose.ToPose2d()});
          }
        } else {
          Logger::FieldDisplay::GetInstance().DisplayPose("LeftEstimatedPose", {});
          Logger::FieldDisplay::GetInstance().DisplayPose("DiscardedLeftEstimatedPose", {});
        }

        auto rightPose = estimatePoses[SubVision::Right];
        if (rightPose.has_value()){
          if(SubVision::GetInstance().IsEstimateUsable(rightPose.value())) {
            auto estimatedPose = rightPose.value();
            double d = SubVision::GetInstance().GetDev(estimatedPose);
            wpi::array<double,3> dev = {d, d, 0.9};
            PoseHandler::GetInstance().AddVisionMeasurement(
                estimatedPose.estimatedPose.ToPose2d(), estimatedPose.timestamp, dev);
            Logger::FieldDisplay::GetInstance().DisplayPose("RightEstimatedPose",
                                                    estimatedPose.estimatedPose.ToPose2d());
            } else {
              Logger::FieldDisplay::GetInstance().DisplayPose("DiscardedRightEstimatedPose", {rightPose.value().estimatedPose.ToPose2d()});
            }
        } else {
          Logger::FieldDisplay::GetInstance().DisplayPose("RightEstimatedPose", {});
          Logger::FieldDisplay::GetInstance().DisplayPose("DiscardedRightEstimatedPose", {});
        }
      
      },
      {&SubVision::GetInstance()}).IgnoringDisable(true);
}

}