#include <frc2/command/Commands.h>

namespace cmd {
    frc2::CommandPtr DefaultAuton();

    frc2::CommandPtr TESTDriveInASquare();
    frc2::CommandPtr TESTForward250cm();
    frc2::CommandPtr TESTForward250cmWhileTurning();

    frc2::CommandPtr NeutralScoreAndClimb_LeftBump();
    frc2::CommandPtr NeutralScoreAndClimb_LeftTrench();
    frc2::CommandPtr NeutralScoreAndClimb_RightBump();
    frc2::CommandPtr NeutralScoreAndClimb_RightTrench();

    frc2::CommandPtr Hoard_LeftBump();
    frc2::CommandPtr Hoard_LeftTrench();
    frc2::CommandPtr Hoard_RightBump();
    frc2::CommandPtr Hoard_RightTrench();
    
    frc2::CommandPtr OutpostDepotClimb();
}
