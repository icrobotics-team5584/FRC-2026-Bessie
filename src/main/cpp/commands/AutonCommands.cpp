#include "commands/AutonCommands.h"
#include "commands/FuelCommands.h"
#include "subsystems/SubDrivebase.h"
#include "subsystems/SubIntake.h"
#include "subsystems/SubShooter.h"
#include "subsystems/SubHood.h"

namespace cmd {
    frc2::CommandPtr DefaultAuton() {
        return frc2::cmd::Print("Default Auton");
    }

    frc2::CommandPtr SOTMTest() {
        return frc2::cmd::Sequence(
            SubHood::GetInstance().ZeroHood(),
            cmd::ShootOnTheMove().AlongWith(
            SubDrivebase::GetInstance().DriveToPose([] { return frc::Pose2d{2.6_m, 5.8_m, 90_deg}; }, 2.0, 1_m, 30_deg)),
            SubDrivebase::GetInstance().DriveToPose([] { return frc::Pose2d{2.6_m, 2.2_m, -90_deg}; }, 2.0, 1_m, 30_deg),
            SubDrivebase::GetInstance().DriveToPose([] { return frc::Pose2d{2.6_m, 5.8_m, 90_deg}; }, 2.0, 1_m, 30_deg),
            SubDrivebase::GetInstance().DriveToPose([] { return frc::Pose2d{2.6_m, 2.2_m, -90_deg}; }, 2.0, 1_m, 30_deg)
        );
    }
}