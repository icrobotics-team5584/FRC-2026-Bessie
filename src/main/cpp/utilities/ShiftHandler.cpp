#include "utilities/ShiftHandler.h"


RebuiltShift ShiftHandler::GetShift()
{
    if(frc::DriverStation::IsAutonomousEnabled()) {
        return RebuiltShift::AUTON;
    }
    units::second_t secondsPassed = frc::DriverStation::GetMatchTime();
    if(secondsPassed < 10_s) {
        return RebuiltShift::TRANS;
    }

    RebuiltShift winningShift = GetWinningShift();
    RebuiltShift losingShift = winningShift == RebuiltShift::BLUE ? RebuiltShift::BLUE : RebuiltShift::RED;
    if(secondsPassed < 35_s) { /* Shift 1 */
        return losingShift;
    }
    if(secondsPassed < 60_s) { /* Shift 2 */
        return winningShift;
    }
    if(secondsPassed < 75_s) { /* Shift 3 */
        return losingShift;
    }
    if(secondsPassed < 100_s) { /* Shift 4 */
        return winningShift;
    }
    return RebuiltShift::ENDGAME;
}

RebuiltShift ShiftHandler::GetWinningShift()
{
    std::string data = frc::DriverStation::GetGameSpecificMessage();
    if(data.length() < 0) { /* No winning shift message recieved */
        return RebuiltShift::AUTON;
    }

    switch(data[0]) {
    case 'B':
        return RebuiltShift::BLUE;
    case 'R':
        return RebuiltShift::RED;
    }

    return RebuiltShift::AUTON; /* Corrupt data */
}

units::second_t ShiftHandler::GetTimeLeft()
{
    printf("balh\n");
}

bool ShiftHandler::IsShift(RebuiltShift shift)
{
    printf("balh\n");
}
