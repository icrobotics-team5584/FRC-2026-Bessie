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
    
    frc2::CommandPtr NeutralScoreAndClimb_LeftBump() {
        return frc2::cmd::Sequence(
            //STARTING POSITION: left bump (X 3.58m, Y 5.80m, heading 0 degrees)
            SubDrivebase::GetInstance().DriveToPose([] { return frc::Pose2d{7.8_m, 7.0_m, -90_deg}; }, 1.0), //entry to neutral zone
            SubDrivebase::GetInstance().DriveToPose([] { return frc::Pose2d{7.8_m, 4.75_m, -90_deg}; }, 1.0), //exit from neutral zone
            SubDrivebase::GetInstance().DriveToPose([] { return frc::Pose2d{3.58_m, 5.2_m, 180_deg}; }, 1.0), //re-entry to alliance zone (after bump)
            SubDrivebase::GetInstance().DriveToPose([] { return frc::Pose2d{1.65_m, 3.75_m, 0_deg}; }, 1.0) //tower
        );
    }
}