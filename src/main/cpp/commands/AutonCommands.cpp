#include "commands/AutonCommands.h"
#include "commands/FuelCommands.h"
#include "commands/TurretCommands.h"
#include "subsystems/SubDrivebase.h"
#include "subsystems/SubIntake.h"
#include "subsystems/SubShooter.h"
#include "subsystems/SubHood.h"
#include "utilities/PoseHandler.h"
#include "utilities/FieldConstants.h"

namespace cmd {
    frc2::CommandPtr DefaultAuton() {
        return frc2::cmd::Print("Default Auton");
    }

    frc2::CommandPtr TESTDriveInASquare() {
        return frc2::cmd::Sequence(
            SubDrivebase::GetInstance().DriveToPose([] { return frc::Pose2d{0_m, 0_m, 0_deg}; }, 1.0),
            SubDrivebase::GetInstance().DriveToPose([] { return frc::Pose2d{1_m, 0_m, 90_deg}; }, 1.0),
            SubDrivebase::GetInstance().DriveToPose([] { return frc::Pose2d{1_m, 1_m, 180_deg}; }, 1.0),
            SubDrivebase::GetInstance().DriveToPose([] { return frc::Pose2d{0_m, 1_m, 270_deg}; }, 1.0),
            SubDrivebase::GetInstance().DriveToPose([] { return frc::Pose2d{0_m, 0_m, 0_deg}; }, 1.0)
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
            // STARTING POSITION: START_BUMP_LEFT (X 3.58m, Y 5.80m, heading 0 degrees)
            SubHood::GetInstance().ZeroHood(),
            cmd::ShootOnTheMove().WithTimeout(2_s)
        ).AndThen(frc2::cmd::Sequence(
            SubDrivebase::GetInstance().DriveOverBump(frc::ChassisSpeeds{3_mps, 0_mps, 0_tps}),
            frc2::cmd::RunOnce([] {
                SubDrivebase::GetInstance().SetPose(frc::Pose2d{5.7_m, 5.8_m, SubDrivebase::GetInstance().GetGyroAngle()});
            }), //reset position after traversing the bump

            SubDrivebase::GetInstance().DriveToPose([] { return frc::Pose2d{6.5_m, 5.5_m, -90_deg}; }, 1.0, 5_cm, 5_deg), //TEMP drive to intake pos
            //REPLACE ABOVE WITH: SubDrivebase::GetInstance().DriveToPose([] { return fieldpos::NEUTRAL_IN_LEFT; }, 1.0),
 
            SubDrivebase::GetInstance().DriveToPose([] { return frc::Pose2d{6.5_m, 3.5_m, -90_deg}; }, 0.5, 20_cm, 5_deg) //TEMP drive to end intake pos
                .DeadlineFor(SubIntake::GetInstance().IntakeOn()),
            //REPLACE ABOVE WITH: SubDrivebase::GetInstance().DriveToPose([] { return fieldpos::NEUTRAL_END_LEFT; }, 0.5, 20_cm),

            SubDrivebase::GetInstance().DriveToPose([] { return frc::Pose2d{5.9_m, 5.3_m, 180_deg}; }, 1.0, 20_cm, 5_deg)
                .DeadlineFor(SubIntake::GetInstance().IntakeOn()), //return to bump (continue intaking)
            
            SubDrivebase::GetInstance().DriveOverBump(frc::ChassisSpeeds{-3_mps, 0_mps, 0_tps}),
            frc2::cmd::RunOnce([] {
                SubDrivebase::GetInstance().SetPose(frc::Pose2d{3.5_m, 5.3_m, SubDrivebase::GetInstance().GetGyroAngle()});
            }) //reset position again
        ).DeadlineFor(
            cmd::AimAtSpot(fieldpos::HUB_POSITION.ToTranslation2d())
        )).AndThen(frc2::cmd::Sequence(
            SubDrivebase::GetInstance().DriveToPose([] { return fieldpos::TOWER; }, 1.0)
                .AlongWith(cmd::ShootOnTheMove().WithTimeout(6_s))//, 
            
            //SubClimber::GetInstance().ClimbL1()
        ));
    }

    //DO NOT RUN UNTIL WE HAVE FULL HALF FIELD
    frc2::CommandPtr NeutralScoreAndClimb_LeftTrench() {
        return frc2::cmd::Sequence(
            // STARTING POSITION: START_TRENCH_LEFT (X 3.58m, Y 7.50m, heading 0 degrees)
            SubHood::GetInstance().ZeroHood(),
            cmd::ShootOnTheMove().WithTimeout(2_s)
        ).AndThen(frc2::cmd::Sequence(
            SubDrivebase::GetInstance().DriveToPose([] { return fieldpos::NEUTRAL_IN_LEFT; }, 1.0),
 
            SubDrivebase::GetInstance().DriveToPose([] { return fieldpos::NEUTRAL_END_LEFT; }, 0.5, 20_cm)
                .DeadlineFor(SubIntake::GetInstance().IntakeOn()),

            SubDrivebase::GetInstance().DriveToPose([] { return frc::Pose2d{5.9_m, 5.3_m, 180_deg}; }, 1.0, 20_cm, 5_deg)
                .DeadlineFor(SubIntake::GetInstance().IntakeOn()), //return to bump (continue intaking)
            
            SubDrivebase::GetInstance().DriveOverBump(frc::ChassisSpeeds{-3_mps, 0_mps, 0_tps}),
            frc2::cmd::RunOnce([] {
                SubDrivebase::GetInstance().SetPose(frc::Pose2d{3.5_m, 5.3_m, SubDrivebase::GetInstance().GetGyroAngle()});
            }) //reset position
        ).DeadlineFor(
            cmd::AimAtSpot(fieldpos::HUB_POSITION.ToTranslation2d())
        )).AndThen(frc2::cmd::Sequence(
            SubDrivebase::GetInstance().DriveToPose([] { return fieldpos::TOWER; }, 1.0)
                .AlongWith(cmd::ShootOnTheMove().WithTimeout(6_s))//, 
            
            //SubClimber::GetInstance().ClimbL1()
        ));
    }

    //DO NOT RUN UNTIL WE HAVE FULL HALF FIELD
    frc2::CommandPtr NeutralScoreAndClimb_RightBump() {
        return frc2::cmd::Sequence(
            // STARTING POSITION: START_BUMP_RIGHT (X 3.58m, Y 2.27m, heading 0 degrees)
            SubHood::GetInstance().ZeroHood(),
            cmd::ShootOnTheMove().WithTimeout(2_s)
        ).AndThen(frc2::cmd::Sequence(
            SubDrivebase::GetInstance().DriveOverBump(frc::ChassisSpeeds{3_mps, 0_mps, 0_tps}),
            frc2::cmd::RunOnce([] {
                SubDrivebase::GetInstance().SetPose(frc::Pose2d{5.7_m, 2.27_m, SubDrivebase::GetInstance().GetGyroAngle()});
            }), //reset position after traversing the bump

            SubDrivebase::GetInstance().DriveToPose([] { return fieldpos::NEUTRAL_IN_RIGHT; }, 1.0),
 
            SubDrivebase::GetInstance().DriveToPose([] { return fieldpos::NEUTRAL_END_RIGHT; }, 0.5, 20_cm)
                .DeadlineFor(SubIntake::GetInstance().IntakeOn()),

            SubDrivebase::GetInstance().DriveToPose([] { return frc::Pose2d{5.9_m, 2.9_m, 180_deg}; }, 1.0, 20_cm, 5_deg)
                .DeadlineFor(SubIntake::GetInstance().IntakeOn()), //return to bump (continue intaking)
            
            SubDrivebase::GetInstance().DriveOverBump(frc::ChassisSpeeds{-3_mps, 0_mps, 0_tps}),
            frc2::cmd::RunOnce([] {
                SubDrivebase::GetInstance().SetPose(frc::Pose2d{3.5_m, 2.9_m, SubDrivebase::GetInstance().GetGyroAngle()});
            }) //reset position again
        ).DeadlineFor(
            cmd::AimAtSpot(fieldpos::HUB_POSITION.ToTranslation2d())
        )).AndThen(frc2::cmd::Sequence(
            SubDrivebase::GetInstance().DriveToPose([] { return fieldpos::TOWER; }, 1.0)
                .AlongWith(cmd::ShootOnTheMove().WithTimeout(6_s))//, 
            
            //SubClimber::GetInstance().ClimbL1()
        ));
    }

    //DO NOT RUN UNTIL WE HAVE FULL HALF FIELD
    frc2::CommandPtr NeutralScoreAndClimb_RightTrench() {
        return frc2::cmd::Sequence(
            // STARTING POSITION: START_TRENCH_RIGHT (X 3.58m, Y 0.57m, heading 0 degrees)
            SubHood::GetInstance().ZeroHood(),
            cmd::ShootOnTheMove().WithTimeout(2_s)
        ).AndThen(frc2::cmd::Sequence(
            SubDrivebase::GetInstance().DriveToPose([] { return fieldpos::NEUTRAL_IN_RIGHT; }, 1.0),
 
            SubDrivebase::GetInstance().DriveToPose([] { return fieldpos::NEUTRAL_END_RIGHT; }, 0.5, 20_cm)
                .DeadlineFor(SubIntake::GetInstance().IntakeOn()),

            SubDrivebase::GetInstance().DriveToPose([] { return frc::Pose2d{5.9_m, 2.9_m, 180_deg}; }, 1.0, 20_cm, 5_deg)
                .DeadlineFor(SubIntake::GetInstance().IntakeOn()), //return to bump (continue intaking)
            
            SubDrivebase::GetInstance().DriveOverBump(frc::ChassisSpeeds{-3_mps, 0_mps, 0_tps}),
            frc2::cmd::RunOnce([] {
                SubDrivebase::GetInstance().SetPose(frc::Pose2d{3.5_m, 2.9_m, SubDrivebase::GetInstance().GetGyroAngle()});
            }) //reset position
        ).DeadlineFor(
            cmd::AimAtSpot(fieldpos::HUB_POSITION.ToTranslation2d())
        )).AndThen(frc2::cmd::Sequence(
            SubDrivebase::GetInstance().DriveToPose([] { return fieldpos::TOWER; }, 1.0)
                .AlongWith(cmd::ShootOnTheMove().WithTimeout(6_s))//, 
            
            //SubClimber::GetInstance().ClimbL1()
        ));
    }
}