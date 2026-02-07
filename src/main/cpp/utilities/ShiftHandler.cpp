#include "utilities/ShiftHandler.h"

#include "utilities/Logger.h"

RebuiltShift ShiftHandler::GetCurrentShift() {
  if (frc::DriverStation::IsAutonomousEnabled()) {
    return RebuiltShift::AUTON;
  }
  units::second_t secondsPassed = frc::DriverStation::GetMatchTime();
  if (secondsPassed == -1_s) {
    return RebuiltShift::NONE;
  }

  if (secondsPassed > 130_s) {
    return RebuiltShift::TRANS;
  }

  RebuiltShift losingShift, winningShift = GetWinningShift();
  if (winningShift == RebuiltShift::NONE) {
    losingShift = RebuiltShift::NONE;
  } else { /* filps RebuiltShift::BLUE to RebuiltShift::RED and vice versa*/
    losingShift = (winningShift == RebuiltShift::BLUE ? RebuiltShift::RED : RebuiltShift::BLUE);
  }
  if (secondsPassed > 105_s) { /* Shift 1 */
    return losingShift;
  }
  if (secondsPassed > 80_s) { /* Shift 2 */
    return winningShift;
  }
  if (secondsPassed > 55_s) { /* Shift 3 */
    return losingShift;
  }
  if (secondsPassed > 30_s) { /* Shift 4 */
    return winningShift;
  }
  return RebuiltShift::ENDGAME;
}

RebuiltShift ShiftHandler::GetWinningShift() {
  std::string data = frc::DriverStation::GetGameSpecificMessage();
  if (data.length() == 0) { /* No winning shift message recieved */
    return RebuiltShift::NONE;
  }

  switch (data[0]) {
    case 'B':
      return RebuiltShift::BLUE;
    case 'R':
      return RebuiltShift::RED;
  }

  return RebuiltShift::NONE; /* Corrupt data */
}

units::second_t ShiftHandler::GetTimeLeft() {
  units::second_t matchTime = frc::DriverStation::GetMatchTime();
  if (frc::DriverStation::IsAutonomousEnabled()) {
    return matchTime;
  }

  if (matchTime > 130_s) {
    return matchTime - 130_s;
  }
  if (matchTime > 105_s) { /* Shift 1 */
    return matchTime - 105_s;
  }
  if (matchTime > 80_s) { /* Shift 2 */
    return matchTime - 80_s;
  }
  if (matchTime > 55_s) { /* Shift 3 */
    return matchTime - 55_s;
  }
  if (matchTime > 30_s) { /* Shift 4 */
    return matchTime - 30_s;
  }
  return matchTime;
}

std::string ShiftHandler::GetShiftName(RebuiltShift shift) {
  switch (shift) {
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
    default:
      return "None";
  }
}

bool ShiftHandler::IsShift(RebuiltShift shift) {
  return GetCurrentShift() == shift ? true : false; /* check if matching */
}


bool ShiftHandler::IsActiveShift(bool overrideActive = false) {
  if (overrideActive) {
    return true;
  }
  if (frc::DriverStation::GetMatchType() == frc::DriverStation::MatchType::kNone) {
    return true;
  }
  RebuiltShift currentShift = GetCurrentShift();
  RebuiltShift myShift = (RebuiltShift)frc::DriverStation::GetAlliance().value_or(frc::DriverStation::Alliance::kBlue);

  if (currentShift == RebuiltShift::AUTON || currentShift == RebuiltShift::TRANS ||
      currentShift == RebuiltShift::ENDGAME || myShift == currentShift) {
    return true;
  }

  return false;
}