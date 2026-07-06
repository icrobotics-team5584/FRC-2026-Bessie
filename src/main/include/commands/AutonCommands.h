#include <frc2/command/Commands.h>
#include <frc/geometry/Pose2d.h>
#include <units/length.h>
#include <units/angle.h>

namespace cmd {
frc2::CommandPtr TESTDriveInASquare();
frc2::CommandPtr TESTForward250cm();
frc2::CommandPtr TESTForward250cmWhileTurning();

frc2::CommandPtr Hoard_RightBump();
frc2::CommandPtr Hoard_LeftBump();
frc2::CommandPtr Hoard_RightTrench(bool flip = false);
frc2::CommandPtr Hoard_LeftTrench();

frc2::CommandPtr NeutralOnePassAndOutpost_RightBump();
frc2::CommandPtr NeutralOnePassAndOutpost_RightTrench();
frc2::CommandPtr NeutralOnePassAndOutpost_RightTrench_NoBump();

frc2::CommandPtr NeutralTwoPassToOutpost_RightTrench();

frc2::CommandPtr NeutralTwoPassToMid_RightTrench(bool flip = false);
frc2::CommandPtr NeutralTwoPassToMid_LeftTrench();

frc2::CommandPtr ShortFirstPass_NeutralTwoPassToMid_RightTrench(bool flip = false);
frc2::CommandPtr ShortFirstPass_NeutralTwoPassToMid_LeftTrench();

frc2::CommandPtr ShootAndStay();

frc2::CommandPtr AutoZeroGyroFromVision();

frc2::CommandPtr AutonomousShoot(units::second_t shootTime);

frc2::CommandPtr AutonomousDriveTo(frc::Pose2d targetPose, bool flipY, double speedScaling = 1,
  units::meter_t posErrorTolerance = 2_cm, units::degree_t rotErrorTolerance = 2_deg);
}  // namespace cmd