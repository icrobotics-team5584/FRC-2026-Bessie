#include "utilities/ShiftHandler.h"


RebuiltShift ShiftHandler::GetCurrentShift()
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
    if(secondsPassed < 85_s) { /* Shift 3 */
        return losingShift;
    }
    if(secondsPassed < 110_s) { /* Shift 4 */
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
    units::second_t matchTime = frc::DriverStation::GetMatchTime();
    if(frc::DriverStation::IsAutonomousEnabled()) {
        return 20_s - matchTime;
    }

    if(matchTime < 10_s) {
        return 10_s - matchTime;
    }
    if(matchTime < 35_s) { /* Shift 1 */
        return 35_s - matchTime;
    }
    if(matchTime < 60_s) { /* Shift 2 */
        return 60_s - matchTime;
    }
    if(matchTime < 85_s) { /* Shift 3 */
        return 78_s - matchTime;
    }
    if(matchTime < 110_s) { /* Shift 4 */
        return 110_s - matchTime;
    }
    return 140_s - matchTime;
}

std::string GetShiftName(RebuiltShift shift)
{
    switch(shift) {
    case RebuiltShift::AUTON:
        return "Autonomous";
    case RebuiltShift::TRANS:
        return "Transition";
    case RebuiltShift::BLUE:
        return "Blue";
    case RebuiltShift::RED:
        return "Red";
    case RebuiltShift::ENDGAME:
        return "Endgame";
    }
}

bool ShiftHandler::IsShift(RebuiltShift shift)
{
    return GetCurrentShift() == shift ? true : false; /* check if matching */
}

bool ShiftHandler::IsActiveShift()
{
    units::second_t matchTime = frc::DriverStation::GetMatchTime();
    RebuiltShift currentShift = GetCurrentShift();
    frc::DriverStation::Alliance allicance = frc::DriverStation::GetAlliance().value_or(frc::DriverStation::Alliance::kBlue);
    bool wonAutonShift = (allicance == frc::DriverStation::Alliance::kBlue) && (currentShift == RebuiltShift::BLUE);
    bool shift2 = 60_s > matchTime && matchTime > 35_s;     /* 35s - 60s */
    bool shift4 = 110_s > matchTime && matchTime > 85_s;    /* 85 - 110 */

    if(
        currentShift == RebuiltShift::AUTON ||
        currentShift == RebuiltShift::TRANS ||
        currentShift == RebuiltShift::ENDGAME
    ) {
        return true;
    }

    if(wonAutonShift && (shift2||shift4)) {
        return true;
    }

    return false;
}
