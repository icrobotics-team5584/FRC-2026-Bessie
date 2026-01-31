#include <frc2/command/Commands.h>
#include <units/angle.h>
#include <frc/geometry/Pose2d.h>

namespace cmd {
    frc2::CommandPtr AimAtFieldRelative(std::function<units::degree_t()> target);
    frc2::CommandPtr AimAtSpot(frc::Translation2d target);

    units::meter_t CalcShootOnTheMoveDistance();
    units::degree_t CalcShootOnTheMoveAngle();

    frc::Pose2d CalcFutureTurretPose();

    static constexpr frc::Translation2d TARGETPOSE = frc::Translation2d{12_m, 4_m};
}


