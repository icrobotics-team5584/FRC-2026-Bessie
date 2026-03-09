#include "commands/AutonCommands.h"

#include "subsystems/SubDrivebase.h"
#include "subsystems/Hood/SubHood.h"
#include "subsystems/SubIntake.h"
#include "subsystems/SubShooter.h"
#include "subsystems/SubDeploy.h"
//#include "subsystems/SubClimber.h"

#include "commands/FuelCommands.h"
#include "commands/TurretCommands.h"

#include "utilities/FieldConstants.h"
#include "utilities/PoseHandler.h"

namespace cmd {

    /* TESTING AUTONS */

    frc2::CommandPtr TESTDriveInASquare() {
        return frc2::cmd::Sequence(
            SubDrivebase::GetInstance().DriveToPose([] { return frc::Pose2d{0_m, 0_m, 0_deg}; }, 1.0, 80_cm, 15_deg),
            SubDrivebase::GetInstance().DriveToPose([] { return frc::Pose2d{2_m, 0_m, 0_deg}; }, 1.0, 80_cm, 15_deg),
            SubDrivebase::GetInstance().DriveToPose([] { return frc::Pose2d{2_m, 2_m, 0_deg}; }, 1.0, 80_cm, 15_deg),
            SubDrivebase::GetInstance().DriveToPose([] { return frc::Pose2d{0_m, 2_m, 0_deg}; }, 1.0, 80_cm, 15_deg),
            SubDrivebase::GetInstance().DriveToPose([] { return frc::Pose2d{0_m, 0_m, 0_deg}; }, 1.0, 20_cm, 5_deg)
        );
    }

    frc2::CommandPtr TESTForward250cm() {
        return SubDrivebase::GetInstance().DriveToPose([] { return frc::Pose2d{2.5_m, 0.0_m, 0_deg}; }, 1.0);
    }

    frc2::CommandPtr TESTForward250cmWhileTurning() {
        return SubDrivebase::GetInstance().DriveToPose([] { return frc::Pose2d{2.5_m, 0.0_m, 180_deg}; }, 1.0);
    }
    
    /* NEUTRAL ONE PASS AND CLIMB AUTONS */

    // frc2::CommandPtr NeutralOnePassAndClimb_LeftBump() {
    //     return frc2::cmd::Sequence(
    //         // STARTING POSITION: START_BUMP_LEFT (X 3.58m, Y 5.77m, heading 0 degrees)
    //         SubHood::GetInstance().ZeroHood(),
    //         cmd::ShootOnTheMove().WithTimeout(2_s)
    //     ).AndThen(frc2::cmd::Sequence(
    //         SubDrivebase::GetInstance().DriveOverBump(frc::ChassisSpeeds{3_mps, 0_mps, 0_tps}, frc::Translation2d{5.8_m, 5.8_m}), //drive over bump, reset position at the end of the bump

    //         SubDrivebase::GetInstance().DriveToPose([] { return fieldpos::NEUTRAL_ONEPASS_IN_LEFT; }, 1.0, 20_cm),
 
    //         SubDrivebase::GetInstance().DriveToPose([] { return fieldpos::NEUTRAL_ONEPASS_END_LEFT; }, 0.5, 20_cm, 3_deg)
    //             .DeadlineFor(SubIntake::GetInstance().IntakeOn()),

    //         SubDrivebase::GetInstance().DriveToPose([] { return fieldpos::START_REENTRY_BUMP_LEFT; }, 1.0, 20_cm, 5_deg)
    //             .DeadlineFor(SubIntake::GetInstance().IntakeOn()), //return to bump (continue intaking)
            
    //         SubDrivebase::GetInstance().DriveOverBump(frc::ChassisSpeeds{-3_mps, 0_mps, 0_tps}, frc::Translation2d{3.5_m, 5.4_m}) //drive back over bump, reset position at the end of the bump
    //     ).DeadlineFor(
    //         cmd::AimAtHub()
    //     )).AndThen(frc2::cmd::Sequence(
    //         SubDrivebase::GetInstance().DriveToPose([] { return fieldpos::TOWER; }, 1.0)
    //             .AlongWith(cmd::ShootOnTheMove().WithTimeout(6_s))
            
    //         SubClimber::GetInstance().ToggleClimb()
    //     ));
    // }

    // frc2::CommandPtr NeutralOnePassAndClimb_LeftTrench() {
    //     return frc2::cmd::Sequence(
    //         // STARTING POSITION: START_TRENCH_LEFT (X 3.58m, Y 7.47m, heading 0 degrees)
    //         SubHood::GetInstance().ZeroHood(),
    //         cmd::ShootOnTheMove().WithTimeout(2_s)
    //     ).AndThen(frc2::cmd::Sequence(
    //         SubDrivebase::GetInstance().DriveToPose([] { return fieldpos::NEUTRAL_ONEPASS_IN_LEFT; }, 1.0, 20_cm),
 
    //         SubDrivebase::GetInstance().DriveToPose([] { return fieldpos::NEUTRAL_ONEPASS_END_LEFT; }, 0.5, 20_cm, 3_deg)
    //             .DeadlineFor(SubIntake::GetInstance().IntakeOn()),

    //         SubDrivebase::GetInstance().DriveToPose([] { return fieldpos::START_REENTRY_BUMP_LEFT; }, 1.0, 20_cm, 5_deg)
    //             .DeadlineFor(SubIntake::GetInstance().IntakeOn()), //return to bump (continue intaking)
            
    //         SubDrivebase::GetInstance().DriveOverBump(frc::ChassisSpeeds{-3_mps, 0_mps, 0_tps}, frc::Translation2d{3.5_m, 5.4_m}) //drive back over bump, reset position at the end of the bump
    //     ).DeadlineFor(
    //         cmd::AimAtHub()
    //     )).AndThen(frc2::cmd::Sequence(
    //         SubDrivebase::GetInstance().DriveToPose([] { return fieldpos::TOWER; }, 1.0)
    //             .AlongWith(cmd::ShootOnTheMove().WithTimeout(6_s))
            
    //         SubClimber::GetInstance().ToggleClimb()
    //     ));
    // }

    // frc2::CommandPtr NeutralOnePassAndClimb_LeftTrench_NoBump() {
    //     return frc2::cmd::Sequence(
    //         // STARTING POSITION: START_TRENCH_LEFT (X 3.58m, Y 7.47m, heading 0 degrees)
    //         SubHood::GetInstance().ZeroHood(),
    //         cmd::ShootOnTheMove().WithTimeout(2_s)
    //     ).AndThen(frc2::cmd::Sequence(
    //         SubDrivebase::GetInstance().DriveToPose([] { return fieldpos::NEUTRAL_ONEPASS_IN_LEFT; }, 1.0, 20_cm),
 
    //         SubDrivebase::GetInstance().DriveToPose([] { return fieldpos::NEUTRAL_ONEPASS_END_LEFT; }, 0.5, 20_cm, 3_deg)
    //             .DeadlineFor(SubIntake::GetInstance().IntakeOn()),

    //         SubDrivebase::GetInstance().DriveToPose([] { return fieldpos::START_REENTRY_TRENCH_LEFT; }, 1.0, 20_cm, 5_deg)
    //             .DeadlineFor(SubIntake::GetInstance().IntakeOn()), //return to trench (continue intaking)
            
    //         SubDrivebase::GetInstance().DriveToPose([] { return fieldpos::END_REENTRY_TRENCH_LEFT; }, 1.0, 20_cm, 5_deg)
    //     ).DeadlineFor(
    //         cmd::AimAtHub()
    //     )).AndThen(frc2::cmd::Sequence(
    //         SubDrivebase::GetInstance().DriveToPose([] { return fieldpos::TOWER; }, 1.0)
    //             .AlongWith(cmd::ShootOnTheMove().WithTimeout(6_s))
                        
    //         SubClimber::GetInstance().ToggleClimb()
    //     ));
    // }

    // frc2::CommandPtr NeutralOnePassAndClimb_RightBump() {
    //     return frc2::cmd::Sequence(
    //         // STARTING POSITION: START_BUMP_RIGHT (X 3.58m, Y 2.27m, heading 0 degrees)
    //         SubHood::GetInstance().ZeroHood(),
    //         cmd::ShootOnTheMove().WithTimeout(2_s)
    //     ).AndThen(frc2::cmd::Sequence(
    //         SubDrivebase::GetInstance().DriveOverBump(frc::ChassisSpeeds{3_mps, 0_mps, 0_tps}, frc::Translation2d{5.8_m, 2.27_m}), //drive over bump, reset position at the end of the bump

    //         SubDrivebase::GetInstance().DriveToPose([] { return fieldpos::NEUTRAL_ONEPASS_IN_RIGHT; }, 1.0, 20_cm),
 
    //         SubDrivebase::GetInstance().DriveToPose([] { return fieldpos::NEUTRAL_ONEPASS_END_RIGHT; }, 0.5, 20_cm, 3_deg)
    //             .DeadlineFor(SubIntake::GetInstance().IntakeOn()),

    //         SubDrivebase::GetInstance().DriveToPose([] { return fieldpos::START_REENTRY_BUMP_RIGHT; }, 1.0, 20_cm, 5_deg)
    //             .DeadlineFor(SubIntake::GetInstance().IntakeOn()), //return to bump (continue intaking)
            
    //         SubDrivebase::GetInstance().DriveOverBump(frc::ChassisSpeeds{-3_mps, 0_mps, 0_tps}, frc::Translation2d{3.5_m, 2.64_m}) //drive back over bump, reset position at the end of the bump
    //     ).DeadlineFor(
    //         cmd::AimAtHub()
    //     )).AndThen(frc2::cmd::Sequence(
    //         SubDrivebase::GetInstance().DriveToPose([] { return fieldpos::TOWER; }, 1.0)
    //             .AlongWith(cmd::ShootOnTheMove().WithTimeout(6_s))
            
    //         SubClimber::GetInstance().ToggleClimb()
    //     ));
    // }

    // frc2::CommandPtr NeutralOnePassAndClimb_RightTrench() {
    //     return frc2::cmd::Sequence(
    //         // STARTING POSITION: START_TRENCH_RIGHT (X 3.58m, Y 0.57m, heading 0 degrees)
    //         SubHood::GetInstance().ZeroHood(),
    //         cmd::ShootOnTheMove().WithTimeout(2_s)
    //     ).AndThen(frc2::cmd::Sequence(
    //         SubDrivebase::GetInstance().DriveToPose([] { return fieldpos::NEUTRAL_ONEPASS_IN_RIGHT; }, 1.0, 20_cm),
 
    //         SubDrivebase::GetInstance().DriveToPose([] { return fieldpos::NEUTRAL_ONEPASS_END_RIGHT; }, 0.5, 20_cm, 3_deg)
    //             .DeadlineFor(SubIntake::GetInstance().IntakeOn()),

    //         SubDrivebase::GetInstance().DriveToPose([] { return fieldpos::START_REENTRY_BUMP_RIGHT; }, 1.0, 20_cm, 5_deg)
    //             .DeadlineFor(SubIntake::GetInstance().IntakeOn()), //return to bump (continue intaking)
            
    //         SubDrivebase::GetInstance().DriveOverBump(frc::ChassisSpeeds{-3_mps, 0_mps, 0_tps}, frc::Translation2d{3.5_m, 2.64_m}) //drive back over bump, reset position at the end of the bump
    //     ).DeadlineFor(
    //         cmd::AimAtHub()
    //     )).AndThen(frc2::cmd::Sequence(
    //         SubDrivebase::GetInstance().DriveToPose([] { return fieldpos::TOWER; }, 1.0)
    //             .AlongWith(cmd::ShootOnTheMove().WithTimeout(6_s))
            
    //         SubClimber::GetInstance().ToggleClimb()
    //     ));
    // }

    // frc2::CommandPtr NeutralOnePassAndClimb_RightTrench_NoBump() {
    //     return frc2::cmd::Sequence(
    //         // STARTING POSITION: START_TRENCH_RIGHT (X 3.58m, Y 0.57m, heading 0 degrees)
    //         SubHood::GetInstance().ZeroHood(),
    //         cmd::ShootOnTheMove().WithTimeout(2_s)
    //     ).AndThen(frc2::cmd::Sequence(
    //         SubDrivebase::GetInstance().DriveToPose([] { return fieldpos::NEUTRAL_ONEPASS_IN_RIGHT; }, 1.0, 20_cm),
 
    //         SubDrivebase::GetInstance().DriveToPose([] { return fieldpos::NEUTRAL_ONEPASS_END_RIGHT; }, 0.5, 20_cm, 3_deg)
    //             .DeadlineFor(SubIntake::GetInstance().IntakeOn()),

    //         SubDrivebase::GetInstance().DriveToPose([] { return fieldpos::START_REENTRY_TRENCH_RIGHT; }, 1.0, 20_cm, 5_deg)
    //             .DeadlineFor(SubIntake::GetInstance().IntakeOn()), //return to trench (continue intaking)

    //         SubDrivebase::GetInstance().DriveToPose([] { return fieldpos::END_REENTRY_TRENCH_RIGHT; }, 1.0, 20_cm, 5_deg)
    //     ).DeadlineFor(
    //         cmd::AimAtHub()
    //     )).AndThen(frc2::cmd::Sequence(
    //         SubDrivebase::GetInstance().DriveToPose([] { return fieldpos::TOWER; }, 1.0)
    //             .AlongWith(cmd::ShootOnTheMove().WithTimeout(6_s)) 
            
    //         SubClimber::GetInstance().ToggleClimb()
    //     ));
    // }

    /* HOARD AUTONS */

    frc2::CommandPtr Hoard_LeftBump() {
        return frc2::cmd::Sequence(
            // STARTING POSITION: START_BUMP_LEFT (X 3.58m, Y 5.77m, heading 0 degrees)
            SubHood::GetInstance().ZeroHood(),
            cmd::ShootOnTheMove().WithTimeout(2_s),

            SubDrivebase::GetInstance().DriveOverBump(frc::ChassisSpeeds{3_mps, 0_mps, 0_tps}, frc::Translation2d{5.8_m, 5.8_m}), //drive over bump, reset position at the end of the bump

            SubDrivebase::GetInstance().DriveToPose([] { return fieldpos::NEUTRAL_ONEPASS_IN_LEFT; }, 1.0, 20_cm),
 
            SubDrivebase::GetInstance().DriveToPose([] { return frc::Pose2d{7.8_m, 1.0_m, 270_deg}; }, 0.5, 20_cm, 5_deg)
                .AlongWith(cmd::ShootOnTheMove())
        );
    }

    frc2::CommandPtr Hoard_LeftTrench() {
        return frc2::cmd::Sequence(
            // STARTING POSITION: START_TRENCH_LEFT (X 3.58m, Y 7.47m, heading 0 degrees)
            SubHood::GetInstance().ZeroHood(),
            cmd::ShootOnTheMove().WithTimeout(2_s),

            SubDrivebase::GetInstance().DriveToPose([] { return fieldpos::NEUTRAL_ONEPASS_IN_LEFT; }, 1.0, 20_cm),
 
            SubDrivebase::GetInstance().DriveToPose([] { return frc::Pose2d{7.8_m, 1.0_m, 270_deg}; }, 0.5, 20_cm, 5_deg)
                .AlongWith(cmd::ShootOnTheMove())
        );
    }

    frc2::CommandPtr Hoard_RightBump() {
        return frc2::cmd::Sequence(
            // STARTING POSITION: START_BUMP_RIGHT (X 3.58m, Y 2.27m, heading 0 degrees)
            SubHood::GetInstance().ZeroHood(),
            cmd::ShootOnTheMove().WithTimeout(2_s),

            SubDrivebase::GetInstance().DriveOverBump(frc::ChassisSpeeds{3_mps, 0_mps, 0_tps}, frc::Translation2d{5.8_m, 2.27_m}), //drive over bump, reset position at the end of the bump

            SubDrivebase::GetInstance().DriveToPose([] { return fieldpos::NEUTRAL_ONEPASS_IN_RIGHT; }, 1.0, 20_cm),
 
            SubDrivebase::GetInstance().DriveToPose([] { return frc::Pose2d{7.8_m, 6.0_m, 90_deg}; }, 0.5, 20_cm, 5_deg)
                .AlongWith(cmd::ShootOnTheMove())
        );
    }

    frc2::CommandPtr Hoard_RightTrench() {
        return frc2::cmd::Sequence(
            // STARTING POSITION: START_TRENCH_RIGHT (X 3.58m, Y 0.57m, heading 0 degrees)
            SubHood::GetInstance().ZeroHood(),
            cmd::ShootOnTheMove().WithTimeout(2_s),

            SubDrivebase::GetInstance().DriveToPose([] { return fieldpos::NEUTRAL_ONEPASS_IN_RIGHT; }, 1.0, 20_cm),
 
            SubDrivebase::GetInstance().DriveToPose([] { return frc::Pose2d{7.8_m, 6.0_m, 90_deg}; }, 0.5, 20_cm, 5_deg)
                .AlongWith(cmd::ShootOnTheMove())
        );
    }

    /* NEUTRAL ONE PASS AND OUTPOST AUTONS */

    frc2::CommandPtr NeutralOnePassAndOutpost_RightBump() {
        return frc2::cmd::Sequence(
            // STARTING POSITION: START_BUMP_RIGHT (X 3.58m, Y 2.27m, heading 0 degrees)
            SubHood::GetInstance().ZeroHood(),
            cmd::ShootOnTheMove().WithTimeout(2_s)
        ).AndThen(frc2::cmd::Sequence(
            SubDrivebase::GetInstance().DriveOverBump(frc::ChassisSpeeds{3_mps, 0_mps, 0_tps}, frc::Translation2d{5.8_m, 2.27_m}), //drive over bump, reset position at the end of the bump

            SubDrivebase::GetInstance().DriveToPose([] { return fieldpos::NEUTRAL_ONEPASS_IN_RIGHT; }, 1.0, 20_cm),
 
            SubDrivebase::GetInstance().DriveToPose([] { return fieldpos::NEUTRAL_ONEPASS_END_RIGHT; }, 0.5, 20_cm, 3_deg)
                .DeadlineFor(SubIntake::GetInstance().IntakeOn()),

            SubDrivebase::GetInstance().DriveToPose([] { return fieldpos::START_REENTRY_BUMP_RIGHT; }, 1.0, 20_cm, 5_deg)
                .DeadlineFor(SubIntake::GetInstance().IntakeOn()), //return to bump (continue intaking)
            
            SubDrivebase::GetInstance().DriveOverBump(frc::ChassisSpeeds{-3_mps, 0_mps, 0_tps}, frc::Translation2d{3.5_m, 2.64_m}) //drive back over bump, reset position at the end of the bump
        ).DeadlineFor(
            cmd::AimAtHub()
        )).AndThen(
            SubDrivebase::GetInstance().DriveToPose([] { return fieldpos::OUTPOST; }, 1.0)
                .AlongWith(cmd::ShootOnTheMove())
        );
    }

    frc2::CommandPtr NeutralOnePassAndOutpost_RightTrench() {
        return frc2::cmd::Sequence(
            // STARTING POSITION: START_TRENCH_RIGHT (X 3.58m, Y 0.57m, heading 0 degrees)
            SubHood::GetInstance().ZeroHood(),
            cmd::ShootOnTheMove().WithTimeout(2_s)
        ).AndThen(frc2::cmd::Sequence(
            SubDrivebase::GetInstance().DriveToPose([] { return fieldpos::NEUTRAL_ONEPASS_IN_RIGHT; }, 1.0, 20_cm),
 
            SubDrivebase::GetInstance().DriveToPose([] { return fieldpos::NEUTRAL_ONEPASS_END_RIGHT; }, 0.5, 20_cm, 3_deg)
                .DeadlineFor(SubIntake::GetInstance().IntakeOn()),

            SubDrivebase::GetInstance().DriveToPose([] { return fieldpos::START_REENTRY_BUMP_RIGHT; }, 1.0, 20_cm, 5_deg)
                .DeadlineFor(SubIntake::GetInstance().IntakeOn()), //return to bump (continue intaking)
            
            SubDrivebase::GetInstance().DriveOverBump(frc::ChassisSpeeds{-3_mps, 0_mps, 0_tps}, frc::Translation2d{3.5_m, 2.64_m}) //drive back over bump, reset position at the end of the bump
        ).DeadlineFor(
            cmd::AimAtHub()
        )).AndThen(
            SubDrivebase::GetInstance().DriveToPose([] { return fieldpos::OUTPOST; }, 1.0)
                .AlongWith(cmd::ShootOnTheMove())
        );
    }

    frc2::CommandPtr NeutralOnePassAndOutpost_RightTrench_NoBump() {
        return frc2::cmd::Sequence(
            // STARTING POSITION: START_TRENCH_RIGHT (X 3.58m, Y 0.57m, heading 0 degrees)
            SubHood::GetInstance().ZeroHood(),
            cmd::ShootOnTheMove().WithTimeout(2_s)
        ).AndThen(frc2::cmd::Sequence(
            SubDrivebase::GetInstance().DriveToPose([] { return fieldpos::NEUTRAL_ONEPASS_IN_RIGHT; }, 1.0, 20_cm),
 
            SubDrivebase::GetInstance().DriveToPose([] { return fieldpos::NEUTRAL_ONEPASS_END_RIGHT; }, 0.5, 20_cm, 3_deg)
                .DeadlineFor(SubIntake::GetInstance().IntakeOn()),

            SubDrivebase::GetInstance().DriveToPose([] { return fieldpos::START_REENTRY_TRENCH_RIGHT; }, 1.0, 20_cm, 5_deg)
                .DeadlineFor(SubIntake::GetInstance().IntakeOn()), //return to trench (continue intaking)

            SubDrivebase::GetInstance().DriveToPose([] { return fieldpos::END_REENTRY_TRENCH_RIGHT; }, 1.0, 20_cm, 5_deg)
        ).DeadlineFor(
            cmd::AimAtHub()
        )).AndThen(
            SubDrivebase::GetInstance().DriveToPose([] { return fieldpos::OUTPOST; }, 1.0)
                .AlongWith(cmd::ShootOnTheMove())
        );
    }

    /* NEUTRAL ONE PASS, OUTPOST AND CLIMB AUTONS */

    // frc2::CommandPtr NeutralOnePassOutpostClimb_RightTrench() {
    //     return frc2::cmd::Sequence(
    //         // STARTING POSITION: START_TRENCH_RIGHT (X 3.58m, Y 0.57m, heading 0 degrees)
    //         SubDrivebase::GetInstance().DriveToPose([] { return fieldpos::NEUTRAL_ONEPASS_IN_RIGHT; }, 1.0, 30_cm, 3_deg)
    //             .AlongWith(SubHood::GetInstance().ZeroHood()),
 
    //         SubDrivebase::GetInstance().DriveToPose([] { return fieldpos::NEUTRAL_ONEPASS_END_RIGHT; }, 0.5, 20_cm, 3_deg)
    //             .DeadlineFor(SubIntake::GetInstance().IntakeOn()),

    //         SubDrivebase::GetInstance().DriveToPose([] { return fieldpos::START_REENTRY_BUMP_RIGHT; }, 1.0, 20_cm, 5_deg)
    //             .DeadlineFor(SubIntake::GetInstance().IntakeOn()), //return to bump (continue intaking)
            
    //         SubDrivebase::GetInstance().DriveOverBump(frc::ChassisSpeeds{-3_mps, 0_mps, 0_tps}, frc::Translation2d{3.5_m, 2.64_m}) //drive back over bump, reset position at the end of the bump
    //     ).DeadlineFor(
    //         cmd::AimAtHub()
    //     ).AndThen(
    //         cmd::ShootOnTheMove().WithDeadline(frc2::cmd::Sequence(
    //             SubDrivebase::GetInstance().DriveToPose([] { return fieldpos::OUTPOST; }, 1.0, 20_cm),
    //             frc2::cmd::Wait(3_s),
    //             SubDrivebase::GetInstance().DriveToPose([] { return frc::Pose2d{1.7_m, 2.8_m, 0_deg}; }, 1.0, 40_cm, 5_deg), //so we don't crash into the tower
    //             SubDrivebase::GetInstance().DriveToPose([] { return fieldpos::TOWER; }, 1.0)
    //         ))
    //     ).AndThen(
    //         SubClimber::GetInstance().ToggleClimb()
    //     );
    // }

    // frc2::CommandPtr NeutralOnePassOutpostClimb_RightTrench_NoBump() {
    //     return frc2::cmd::Sequence(
    //         // STARTING POSITION: START_TRENCH_RIGHT (X 3.58m, Y 0.57m, heading 0 degrees)
    //         SubDrivebase::GetInstance().DriveToPose([] { return fieldpos::NEUTRAL_ONEPASS_IN_RIGHT; }, 1.0, 30_cm, 3_deg)
    //             .AlongWith(SubHood::GetInstance().ZeroHood()),
 
    //         SubDrivebase::GetInstance().DriveToPose([] { return fieldpos::NEUTRAL_ONEPASS_END_RIGHT; }, 0.5, 20_cm, 3_deg)
    //             .DeadlineFor(SubIntake::GetInstance().IntakeOn()),

    //         SubDrivebase::GetInstance().DriveToPose([] { return fieldpos::START_REENTRY_TRENCH_RIGHT; }, 1.0, 20_cm, 5_deg)
    //             .DeadlineFor(SubIntake::GetInstance().IntakeOn()), //return to trench (continue intaking)
            
    //         SubDrivebase::GetInstance().DriveToPose([] { return fieldpos::END_REENTRY_TRENCH_RIGHT; }, 1.0, 20_cm, 5_deg)
    //     ).DeadlineFor(
    //         cmd::AimAtHub()
    //     ).AndThen(
    //         cmd::ShootOnTheMove().WithDeadline(frc2::cmd::Sequence(
    //             SubDrivebase::GetInstance().DriveToPose([] { return fieldpos::OUTPOST; }, 1.0, 20_cm),
    //             frc2::cmd::Wait(3_s),
    //             SubDrivebase::GetInstance().DriveToPose([] { return frc::Pose2d{1.7_m, 2.8_m, 0_deg}; }, 1.0, 40_cm, 5_deg), //so we don't crash into the tower
    //             SubDrivebase::GetInstance().DriveToPose([] { return fieldpos::TOWER; }, 1.0)
    //         ))
    //     ).AndThen(
    //         SubClimber::GetInstance().ToggleClimb()
    //     );
    // }

    frc2::CommandPtr NeutralTwoPass_LeftTrench() {
        return frc2::cmd::Sequence(
            // STARTING POSITION: START_TRENCH_LEFT (X 3.58m, Y 7.47m, heading 270 degrees)
            SubDrivebase::GetInstance().DriveToPose([] { return frc::Pose2d{5.80_m, 7.45_m, 260_deg}; }, 1.0, 40_cm, 20_deg) //neutral side of trench
                .AlongWith(SubHood::GetInstance().ZeroHood()),
 
            SubIntake::GetInstance().IntakeOn().AlongWith(
                cmd::AimAtHub()
            ).WithDeadline(frc2::cmd::Sequence(
                SubDrivebase::GetInstance().DriveToPose([] { return frc::Pose2d{8.50_m, 6.97_m, 235_deg}; }, 1.0, 40_cm, 15_deg) //first pass start
                .AlongWith(
                    SubDeploy::GetInstance().Zero().AndThen(SubDeploy::GetInstance().DeployIntake()).AsProxy()
                ),
                SubDrivebase::GetInstance().DriveToPose([] { return frc::Pose2d{8.50_m, 4.04_m, 235_deg}; }, 0.7, 50_cm, 10_deg), //first pass end
                SubDrivebase::GetInstance().DriveToPose([] { return frc::Pose2d{8.15_m, 4.04_m, 115_deg}; }, 1.0, 30_cm, 30_deg), //cross centre line
                SubDrivebase::GetInstance().DriveToPose([] { return frc::Pose2d{7.60_m, 7.45_m, 170_deg}; }, 1.0, 30_cm, 20_deg), //transition
                SubDrivebase::GetInstance().DriveToPose([] { return frc::Pose2d{3.60_m, 7.45_m, 180_deg}; }, 1.0, 15_cm, 10_deg) //back to alliance zone
            )),

            cmd::ShootOnTheMove().WithTimeout(5_s),

            SubIntake::GetInstance().IntakeOn().AlongWith(
                cmd::AimAtHub()
            ).WithDeadline(frc2::cmd::Sequence(
                SubDrivebase::GetInstance().DriveToPose([] { return frc::Pose2d{6.10_m, 7.45_m, 180_deg}; }, 1.0, 30_cm, 20_deg), //neutral side of trench
                SubDrivebase::GetInstance().DriveToPose([] { return frc::Pose2d{6.90_m, 7.27_m, 270_deg}; }, 1.0, 30_cm, 20_deg), //second pass off-hub start
                SubDrivebase::GetInstance().DriveToPose([] { return frc::Pose2d{6.40_m, 3.97_m, 250_deg}; }, 1.0, 40_cm, 20_deg), //second pass off-hub end
                SubDrivebase::GetInstance().DriveToPose([] { return frc::Pose2d{6.20_m, 3.47_m, 110_deg}; }, 1.0, 30_cm, 75_deg), //transition
                SubDrivebase::GetInstance().DriveToPose([] { return frc::Pose2d{5.95_m, 3.47_m, 110_deg}; }, 1.0, 20_cm, 15_deg), //second pass hub-side start
                SubDrivebase::GetInstance().DriveToPose([] { return frc::Pose2d{5.95_m, 6.50_m, 110_deg}; }, 1.0, 20_cm, 20_deg), //second pass hub-side end
                SubDrivebase::GetInstance().DriveToPose([] { return frc::Pose2d{5.95_m, 7.45_m, 0_deg}; }, 1.0, 30_cm, 20_deg), //transition to trench
                SubDrivebase::GetInstance().DriveToPose([] { return frc::Pose2d{3.60_m, 7.45_m, 0_deg}; }, 1.0, 20_cm, 10_deg) //drive under trench
            )),

            cmd::ShootOnTheMove()
        );
    }

    frc2::CommandPtr NeutralTwoPassToOutpost_RightTrench() {
        return frc2::cmd::Sequence(
            // STARTING POSITION: START_TRENCH_RIGHT (X 3.58m, Y 0.57m, heading 90 degrees)
            SubDrivebase::GetInstance().DriveToPose([] { return frc::Pose2d{5.80_m, 0.59_m, 100_deg}; }, 1.0, 40_cm, 20_deg) //neutral side of trench
                .AlongWith(SubHood::GetInstance().ZeroHood()),
 
            SubIntake::GetInstance().IntakeOn().AlongWith(
                cmd::AimAtHub()
            ).WithDeadline(frc2::cmd::Sequence(
                SubDrivebase::GetInstance().DriveToPose([] { return frc::Pose2d{8.50_m, 1.07_m, 125_deg}; }, 1.0, 40_cm, 15_deg) //first pass start
                .AlongWith(
                    SubDeploy::GetInstance().Zero().AndThen(SubDeploy::GetInstance().DeployIntake()).AsProxy()
                ),
                SubDrivebase::GetInstance().DriveToPose([] { return frc::Pose2d{8.50_m, 4.0_m, 125_deg}; }, 0.7, 50_cm, 10_deg), //first pass end
                SubDrivebase::GetInstance().DriveToPose([] { return frc::Pose2d{8.15_m, 4.0_m, 245_deg}; }, 1.0, 30_cm, 30_deg), //cross centre line
                SubDrivebase::GetInstance().DriveToPose([] { return frc::Pose2d{7.60_m, 0.59_m, 190_deg}; }, 1.0, 30_cm, 20_deg), //transition
                SubDrivebase::GetInstance().DriveToPose([] { return frc::Pose2d{3.60_m, 0.59_m, 180_deg}; }, 1.0, 15_cm, 10_deg) //back to alliance zone
            )),

            cmd::ShootOnTheMove().WithTimeout(5_s),

            SubIntake::GetInstance().IntakeOn().AlongWith(
                cmd::AimAtHub()
            ).WithDeadline(frc2::cmd::Sequence(
                SubDrivebase::GetInstance().DriveToPose([] { return frc::Pose2d{6.10_m, 0.59_m, 180_deg}; }, 1.0, 30_cm, 20_deg), //neutral side of trench
                SubDrivebase::GetInstance().DriveToPose([] { return frc::Pose2d{6.90_m, 0.77_m, 90_deg}; }, 1.0, 30_cm, 20_deg), //second pass off-hub start
                SubDrivebase::GetInstance().DriveToPose([] { return frc::Pose2d{6.40_m, 4.07_m, 110_deg}; }, 1.0, 40_cm, 20_deg), //second pass off-hub end
                SubDrivebase::GetInstance().DriveToPose([] { return frc::Pose2d{6.20_m, 4.57_m, 250_deg}; }, 1.0, 30_cm, 75_deg), //transition
                SubDrivebase::GetInstance().DriveToPose([] { return frc::Pose2d{5.95_m, 4.57_m, 250_deg}; }, 1.0, 20_cm, 15_deg), //second pass hub-side start
                SubDrivebase::GetInstance().DriveToPose([] { return frc::Pose2d{5.95_m, 1.54_m, 250_deg}; }, 1.0, 20_cm, 20_deg), //second pass hub-side end
                SubDrivebase::GetInstance().DriveToPose([] { return frc::Pose2d{5.95_m, 0.59_m, 0_deg}; }, 1.0, 30_cm, 20_deg), //transition to trench
                SubDrivebase::GetInstance().DriveToPose([] { return frc::Pose2d{3.60_m, 0.59_m, 0_deg}; }, 1.0, 20_cm, 10_deg) //drive under trench
            )),

            cmd::ShootOnTheMove().AlongWith(
                SubDrivebase::GetInstance().DriveToPose([] { return fieldpos::OUTPOST; }, 1.0, 10_cm)
            )
        );
    }

    frc2::CommandPtr NeutralTwoPassToMid_LeftTrench() {
        return frc2::cmd::Sequence(
            // STARTING POSITION: START_TRENCH_LEFT (X 3.58m, Y 7.47m, heading 270 degrees)
            SubDrivebase::GetInstance().DriveToPose([] { return frc::Pose2d{5.80_m, 7.45_m, 260_deg}; }, 1.0, 50_cm, 20_deg) //neutral side of trench
                .AlongWith(SubHood::GetInstance().ZeroHood()),
 
            SubIntake::GetInstance().IntakeOn().AlongWith(
                cmd::AimAtHub()
            ).WithDeadline(frc2::cmd::Sequence(
                SubDrivebase::GetInstance().DriveToPose([] { return frc::Pose2d{8.50_m, 6.97_m, 235_deg}; }, 1.0, 40_cm, 20_deg) //first pass start
                .AlongWith(
                    SubDeploy::GetInstance().Zero().AndThen(SubDeploy::GetInstance().DeployIntake()).AsProxy()
                ),
                SubDrivebase::GetInstance().DriveToPose([] { return frc::Pose2d{8.50_m, 4.04_m, 235_deg}; }, 0.7, 50_cm, 10_deg), //first pass end
                SubDrivebase::GetInstance().DriveToPose([] { return frc::Pose2d{8.15_m, 4.04_m, 115_deg}; }, 1.0, 30_cm, 30_deg), //cross centre line
                SubDrivebase::GetInstance().DriveToPose([] { return frc::Pose2d{7.60_m, 7.45_m, 170_deg}; }, 1.0, 30_cm, 20_deg), //transition
                SubDrivebase::GetInstance().DriveToPose([] { return frc::Pose2d{3.60_m, 7.45_m, 180_deg}; }, 1.0, 15_cm, 10_deg) //back to alliance zone
            )),

            cmd::ShootOnTheMove().WithTimeout(5_s),

            SubIntake::GetInstance().IntakeOn().AlongWith(
                cmd::AimAtHub()
            ).WithDeadline(frc2::cmd::Sequence(
                SubDrivebase::GetInstance().DriveToPose([] { return frc::Pose2d{6.10_m, 7.45_m, 180_deg}; }, 1.0, 30_cm, 20_deg), //neutral side of trench
                SubDrivebase::GetInstance().DriveToPose([] { return frc::Pose2d{6.90_m, 7.27_m, 270_deg}; }, 1.0, 30_cm, 20_deg), //second pass off-hub start
                SubDrivebase::GetInstance().DriveToPose([] { return frc::Pose2d{6.40_m, 3.97_m, 250_deg}; }, 1.0, 40_cm, 20_deg), //second pass off-hub end
                SubDrivebase::GetInstance().DriveToPose([] { return frc::Pose2d{6.20_m, 3.47_m, 110_deg}; }, 1.0, 30_cm, 75_deg), //transition
                SubDrivebase::GetInstance().DriveToPose([] { return frc::Pose2d{5.95_m, 3.47_m, 110_deg}; }, 1.0, 20_cm, 15_deg), //second pass hub-side start
                SubDrivebase::GetInstance().DriveToPose([] { return frc::Pose2d{5.95_m, 6.50_m, 110_deg}; }, 1.0, 20_cm, 20_deg), //second pass hub-side end
                SubDrivebase::GetInstance().DriveToPose([] { return frc::Pose2d{5.95_m, 7.45_m, 180_deg}; }, 1.0, 30_cm, 20_deg), //transition to trench
                SubDrivebase::GetInstance().DriveToPose([] { return frc::Pose2d{3.60_m, 7.45_m, 180_deg}; }, 1.0, 20_cm, 10_deg) //drive under trench
            )),

            cmd::ShootOnTheMove().WithTimeout(4_s),
            SubDrivebase::GetInstance().DriveToPose([] { return frc::Pose2d{6.10_m, 7.45_m, 180_deg}; }, 1.0, 30_cm, 20_deg), //neutral side of trench
            SubDrivebase::GetInstance().DriveToPose([] { return fieldpos::NEUTRAL_ONEPASS_IN_LEFT; }, 1.0, 20_cm, 5_deg)
        );
    }

    frc2::CommandPtr NeutralTwoPassToMid_RightTrench() {
        return frc2::cmd::Sequence(
            // STARTING POSITION: START_TRENCH_RIGHT (X 3.58m, Y 0.57m, heading 90 degrees)
            SubDrivebase::GetInstance().DriveToPose([] { return frc::Pose2d{5.80_m, 0.59_m, 100_deg}; }, 1.0, 50_cm, 20_deg) //neutral side of trench
                .AlongWith(SubHood::GetInstance().ZeroHood()),
 
            SubIntake::GetInstance().IntakeOn().AlongWith(
                cmd::AimAtHub()
            ).WithDeadline(frc2::cmd::Sequence(
                SubDrivebase::GetInstance().DriveToPose([] { return frc::Pose2d{8.50_m, 1.07_m, 125_deg}; }, 1.0, 40_cm, 20_deg) //first pass start
                .AlongWith(
                    SubDeploy::GetInstance().Zero().AndThen(SubDeploy::GetInstance().DeployIntake()).AsProxy()
                ),
                SubDrivebase::GetInstance().DriveToPose([] { return frc::Pose2d{8.50_m, 4.0_m, 125_deg}; }, 0.7, 50_cm, 10_deg), //first pass end
                SubDrivebase::GetInstance().DriveToPose([] { return frc::Pose2d{8.15_m, 4.0_m, 245_deg}; }, 1.0, 30_cm, 30_deg), //cross centre line
                SubDrivebase::GetInstance().DriveToPose([] { return frc::Pose2d{7.60_m, 0.59_m, 190_deg}; }, 1.0, 30_cm, 20_deg), //transition
                SubDrivebase::GetInstance().DriveToPose([] { return frc::Pose2d{3.60_m, 0.59_m, 180_deg}; }, 1.0, 15_cm, 10_deg) //back to alliance zone
            )),

            cmd::ShootOnTheMove().WithTimeout(5_s),

            SubIntake::GetInstance().IntakeOn().AlongWith(
                cmd::AimAtHub()
            ).WithDeadline(frc2::cmd::Sequence(
                SubDrivebase::GetInstance().DriveToPose([] { return frc::Pose2d{6.10_m, 0.59_m, 180_deg}; }, 1.0, 30_cm, 20_deg), //neutral side of trench
                SubDrivebase::GetInstance().DriveToPose([] { return frc::Pose2d{6.90_m, 0.77_m, 90_deg}; }, 1.0, 30_cm, 20_deg), //second pass off-hub start
                SubDrivebase::GetInstance().DriveToPose([] { return frc::Pose2d{6.40_m, 4.07_m, 110_deg}; }, 1.0, 40_cm, 20_deg), //second pass off-hub end
                SubDrivebase::GetInstance().DriveToPose([] { return frc::Pose2d{6.20_m, 4.57_m, 250_deg}; }, 1.0, 30_cm, 75_deg), //transition
                SubDrivebase::GetInstance().DriveToPose([] { return frc::Pose2d{5.95_m, 4.57_m, 250_deg}; }, 1.0, 20_cm, 15_deg), //second pass hub-side start
                SubDrivebase::GetInstance().DriveToPose([] { return frc::Pose2d{5.95_m, 1.54_m, 250_deg}; }, 1.0, 20_cm, 20_deg), //second pass hub-side end
                SubDrivebase::GetInstance().DriveToPose([] { return frc::Pose2d{5.95_m, 0.59_m, 180_deg}; }, 1.0, 30_cm, 25_deg), //transition to trench
                SubDrivebase::GetInstance().DriveToPose([] { return frc::Pose2d{3.60_m, 0.59_m, 180_deg}; }, 1.0, 20_cm, 5_deg) //drive under trench
            )),

            cmd::ShootOnTheMove().WithTimeout(4_s),
            SubDrivebase::GetInstance().DriveToPose([] { return frc::Pose2d{6.10_m, 0.59_m, 180_deg}; }, 1.0, 30_cm, 20_deg), //neutral side of trench
            SubDrivebase::GetInstance().DriveToPose([] { return fieldpos::NEUTRAL_ONEPASS_IN_RIGHT; }, 1.0, 20_cm, 10_deg)
        );
    }
    
    frc2::CommandPtr ShootAndStay() {
        return frc2::cmd::Sequence(
            SubHood::GetInstance().ZeroHood(),
            cmd::ShootOnTheMove().WithTimeout(5_s)
        );
    }
}