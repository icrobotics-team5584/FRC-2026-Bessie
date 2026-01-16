// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>

namespace cmd {
  /**
   * Take current measurement from the camera, then add the analyzed result as a estimated 2d pose of the robot to drivebase
   */
  frc2::CommandPtr AddVisionMeasurement();
}