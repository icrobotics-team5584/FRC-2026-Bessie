#include <frc2/command/Commands.h>

namespace cmd {
    frc2::CommandPtr DefaultAuton();

    frc2::CommandPtr DriveInASquare();
    frc2::CommandPtr Forward250cm();
    frc2::CommandPtr Forward250cmWhileTurning();

    frc2::CommandPtr NeutralScoreAndClimb_LeftBump();
    frc2::CommandPtr NeutralScoreAndClimb_RightBump();

    frc2::CommandPtr Hoard_LeftBump();
    frc2::CommandPtr Hoard_RightBump();
    
    frc2::CommandPtr OutpostDepotClimb();
}