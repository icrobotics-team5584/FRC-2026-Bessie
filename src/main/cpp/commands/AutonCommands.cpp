#include "commands/AutonCommands.h"
#include "subsystems/SubDrivebase.h"

namespace cmd {
    frc2::CommandPtr DefaultAuton() {
        return frc2::cmd::Print("Default Auton");
    }

    frc2::CommandPtr DriveInASquare() {
        return frc2::cmd::Sequence(
            SubDrivebase::GetInstance().DriveToPose([] { return frc::Pose2d{3.2_m, 4.5_m, -90_deg}; }, 1.0),
            SubDrivebase::GetInstance().DriveToPose([] { return frc::Pose2d{3.2_m, 3.5_m, 180_deg}; }, 1.0),
            SubDrivebase::GetInstance().DriveToPose([] { return frc::Pose2d{2.2_m, 3.5_m, 90_deg}; }, 1.0),
            SubDrivebase::GetInstance().DriveToPose([] { return frc::Pose2d{2.2_m, 4.5_m, 0_deg}; }, 1.0),
            SubDrivebase::GetInstance().DriveToPose([] { return frc::Pose2d{3.2_m, 4.5_m, -90_deg}; }, 1.0)
        );
    }
}