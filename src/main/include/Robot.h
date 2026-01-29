// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <optional>

#include <frc/TimedRobot.h>
#include <frc/PowerDistribution.h>
#include <frc2/command/CommandPtr.h>

#include "RobotContainer.h"
#include "Constants.h"
#include <frc/geometry/Pose3d.h>
#include <networktables/StructArrayTopic.h>



class Robot : public frc::TimedRobot {
 public:
  Robot();
  void RobotPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void DisabledExit() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void AutonomousExit() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TeleopExit() override;
  void TestInit() override;
  void TestPeriodic() override;
  void TestExit() override;

 private:
  std::shared_ptr<frc2::CommandPtr> m_autonomousCommand;


  frc::Pose3d zeroPose = frc::Pose3d{0_m, 0_m, 0_m, frc::Rotation3d{0_deg, 0_deg, 0_deg}};
  frc::Pose3d turretPose = frc::Pose3d{-0.315_m, 0.001_m, 0.553_m, frc::Rotation3d{0_deg, 0_deg, 0_deg}};
  frc::Pose3d hoodPose = frc::Pose3d{-0.315_m, 0.001_m, 0.553_m, frc::Rotation3d{0_deg, 0_deg, 0_deg}};
  wpi::array<frc::Pose3d, 2> _zeroedRobotComponentsArray = {zeroPose, zeroPose};
  wpi::array<frc::Pose3d, 2> _finalRobotComponentsArray = {turretPose, hoodPose};
  nt::StructArrayPublisher<frc::Pose3d> arrayPublisher;

  RobotContainer m_container;

  frc::PowerDistribution m_pdh{canid::PDH, frc::PowerDistribution::ModuleType::kRev};
};
