#include <frc/DriverStation.h>

#include <units/time.h>

enum RebuiltShift {
  NONE,
  AUTON,
  TRANS,
  RED,
  BLUE,
  ENDGAME,
};

class ShiftHandler {
 public:
  static RebuiltShift GetCurrentShift();
  static RebuiltShift GetWinningShift();
  static units::second_t GetTimeLeft();
  static std::string GetShiftName(RebuiltShift shift);
  static bool IsShift(RebuiltShift shift);
  static bool IsActiveShift();

 private:
  static units::second_t _GetMatchTime();
};