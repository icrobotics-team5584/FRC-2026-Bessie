#pragma once
#include <frc2/command/Command.h>
#include <frc/geometry/Translation3d.h>
#include <units/velocity.h>

struct ShootConfig {
  frc::Rotation2d PivotAngle; // For Hood
  frc::Rotation2d Yaw; // For Turret
  units::meters_per_second_t Velocity; // For the ball, not the shooter flywheel
};

namespace cmd {
frc2::CommandPtr AimAndShoot(frc::Translation3d target_pose);
}
