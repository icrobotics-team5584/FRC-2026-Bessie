#include <frc2/command/Commands.h>
#include <frc2/command/Command.h>
#include <units/angle.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Pose3d.h>

namespace cmd {
    frc2::CommandPtr AimAtFieldRelative(std::function<units::degree_t()> target);
    frc2::CommandPtr AimAtSpot(frc::Translation2d target);

    units::meter_t CalcShootOnTheMoveDistance();
    units::degree_t CalcShootOnTheMoveAngle();

    frc::Pose2d CalcFuturePose();
}


