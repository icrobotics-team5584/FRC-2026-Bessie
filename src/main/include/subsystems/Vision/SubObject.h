// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include "frc2/command/Commands.h"

#include "utilities/LimelightHelpers.h"

class SubObject : public frc2::SubsystemBase {
 public:
  SubObject();
  static SubObject& GetInstance() {
    static SubObject inst;
    return inst;
    int targId;
    double targTx; 
    double targTy; 
    double targTA; 
    bool targetFound;
  }
  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

frc2::CommandPtr UpdateLargestTarget();
frc2::CommandPtr SwitchPipeline(int pipeline);


 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  std::string limelightName = "limelight";
};
