#include <frc2/command/Commands.h>
#include <units/angle.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Pose3d.h>

namespace cmd {
    frc2::CommandPtr AimAtFieldRelative(std::function<units::degree_t()> target);
    frc2::CommandPtr AimAtPose(frc::Pose2d pose);
    frc2::CommandPtr ShootOnTheMove(frc::Pose3d ShooterTarget);
}
