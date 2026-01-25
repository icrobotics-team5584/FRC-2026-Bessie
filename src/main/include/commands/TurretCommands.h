#include <frc2/command/Commands.h>
#include <units/angle.h>
#include <frc/geometry/Translation2d.h>

namespace cmd {
    frc2::CommandPtr AimAtFieldRelative(std::function<units::degree_t()> target);
    frc2::CommandPtr AimAtSpot(frc::Translation2d target);
}