#include <frc2/command/Command.h>
#include <frc/geometry/Translation3d.h>

namespace cmd {
  /**
   * Take current measurement from the camera, then add the analyzed result as a estimated 2d pose of the robot to drivebase
   */
  frc2::CommandPtr AimAndShoot(frc::Translation3d target_pose);
}