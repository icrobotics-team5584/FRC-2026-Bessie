#include <frc2/command/Commands.h>
#include <frc/geometry/Pose2d.h>
#include <units/length.h>
#include <units/angle.h>

enum AutonomousSide {
    RIGHT = 0,
    LEFT = 1
};

namespace cmd {
frc2::CommandPtr TESTDriveInASquare();
frc2::CommandPtr TESTForward250cm();
frc2::CommandPtr TESTForward250cmWhileTurning();

frc2::CommandPtr Hoard_LeftBump();
frc2::CommandPtr Hoard_LeftTrench();
frc2::CommandPtr Hoard_RightBump();
frc2::CommandPtr Hoard_RightTrench();

frc2::CommandPtr NeutralOnePassAndOutpost_RightBump();
frc2::CommandPtr NeutralOnePassAndOutpost_RightTrench();
frc2::CommandPtr NeutralOnePassAndOutpost_RightTrench_NoBump();

frc2::CommandPtr NeutralTwoPassToOutpost_RightTrench();
frc2::CommandPtr NeutralTwoPassToMid(AutonomousSide side);
frc2::CommandPtr ShortFirstPass_NeutralTwoPassToMid_LeftTrench();
frc2::CommandPtr ShortFirstPass_NeutralTwoPassToMid_RightTrench();

frc2::CommandPtr ShootAndStay();

frc2::CommandPtr AutoZeroGyroFromVision();

frc2::CommandPtr AutonomousShoot(units::second_t shootTime);

// Values from the AutonomousSide enum can be used in the `flipY` parameter to automatically flip autonomous paths across the width of the field. To do so, `targetPose` should be given as a RIGHT-SIDE pose (y-coordinate less than half of field width). `AutonomousSide::RIGHT` is equivalent to 0, or `false`, meaning the path will not be flipped. Conversely, `AutonomousSide::LEFT` is equivalent to 1, or `true`, meaning the path will be flipped across the field width.
frc2::CommandPtr AutonomousDriveTo(frc::Pose2d targetPose, double speedScaling = 1,
  units::meter_t posErrorTolerance = 2_cm, units::degree_t rotErrorTolerance = 2_deg,
  bool flipY = false);
}  // namespace cmd