#include <frc2/command/Command.h>
#include <frc/geometry/Translation3d.h>
#include <units/velocity.h>

struct ShootConfig {
  frc::Rotation2d PivotAngle; // For Hood
  frc::Rotation2d Yaw; // For Turret
  units::meters_per_second_t Velocity; // For the ball, not the shooter flywheel
};

ShootConfig CalShootOnMove(double shooter_h, frc::Translation3d target, frc::Rotation2d piv,
                               units::meters_per_second_t bot_x, units::meters_per_second_t bot_y);

namespace cmd {
  /**
   * Take current measurement from the camera, then add the analyzed result as a estimated 2d pose of the robot to drivebase
   */
  frc2::CommandPtr AimAndShoot(frc::Translation3d target_pose);
}