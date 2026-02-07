#include <frc/DriverStation.h>

#include <units/time.h>

enum RebuiltShift {
  RED = 0, /* set to match frc::DriverStation::Alliance */
  BLUE = 1,
  AUTON = 2,
  TRANS = 3,
  ENDGAME = 4,
  NONE = 5,
};

class ShiftHandler {
 public:
  static RebuiltShift GetCurrentShift();
  static RebuiltShift GetWinningShift();
  static units::second_t GetTimeLeft();
  static std::string GetShiftName(RebuiltShift shift);
  static bool IsShift(RebuiltShift shift);
  static bool IsActiveShift(bool overrideActive);
};