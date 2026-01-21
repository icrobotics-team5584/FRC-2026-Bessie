#include "commands/AutonCommands.h"
#include "subsystems/SubDrivebase.h"

namespace cmd {
    frc2::CommandPtr DefaultAuton() {
        return frc2::cmd::Print("Default Auton");
    }

    frc2::CommandPtr TESTDriveInASquare() {
        return frc2::cmd::Sequence(
            SubDrivebase::GetInstance().DriveToPose([] { return frc::Pose2d{0_m, 0_m, 0_deg}; }, 1.0, 5_cm, 2_deg),
            SubDrivebase::GetInstance().DriveToPose([] { return frc::Pose2d{1_m, 0_m, 90_deg}; }, 1.0, 5_cm, 2_deg),
            SubDrivebase::GetInstance().DriveToPose([] { return frc::Pose2d{1_m, 1_m, 180_deg}; }, 1.0, 5_cm, 2_deg),
            SubDrivebase::GetInstance().DriveToPose([] { return frc::Pose2d{0_m, 1_m, 270_deg}; }, 1.0, 5_cm, 2_deg),
            SubDrivebase::GetInstance().DriveToPose([] { return frc::Pose2d{0_m, 0_m, 0_deg}; }, 1.0, 5_cm, 2_deg)
        );
    }

    frc2::CommandPtr TESTForward250cm() {
        return SubDrivebase::GetInstance().DriveToPose([] { return frc::Pose2d{2.5_m, 0.0_m, 0_deg}; }, 1.0);
    }

    frc2::CommandPtr TESTForward250cmWhileTurning() {
        return SubDrivebase::GetInstance().DriveToPose([] { return frc::Pose2d{2.5_m, 0.0_m, 180_deg}; }, 1.0);
    }
    
    frc2::CommandPtr NeutralScoreAndClimb_LeftBump() {
        return frc2::cmd::Sequence(
            //STARTING POSITION: left bump (X 3.58m, Y 5.80m, heading 0 degrees)
            SubDrivebase::GetInstance().DriveToPose([] { return frc::Pose2d{7.8_m, 7.0_m, -90_deg}; }, 1.0), //entry to neutral zone
            SubDrivebase::GetInstance().DriveToPose([] { return frc::Pose2d{7.8_m, 4.75_m, -90_deg}; }, 1.0, 20_cm), //exit from neutral zone
            SubDrivebase::GetInstance().DriveToPose([] { return frc::Pose2d{3.58_m, 5.2_m, 180_deg}; }, 1.0, 20_cm), //re-entry to alliance zone (after bump)
            SubDrivebase::GetInstance().DriveToPose([] { return frc::Pose2d{1.65_m, 3.75_m, 0_deg}; }, 1.0) //tower
        );
    }
}