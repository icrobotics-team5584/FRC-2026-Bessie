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
  static ShiftHandler& GetInstance() {
    static ShiftHandler inst;
    return inst;
  }

  RebuiltShift GetCurrentShift();
  RebuiltShift GetWinningShift();
  units::second_t GetTimeLeft();
  std::string GetShiftName(RebuiltShift shift);
  bool IsShift(RebuiltShift shift);
  bool IsActiveShift(bool overrideActive=false);

  /*Delete assignment and copy so people don't accidently create copies*/
  ShiftHandler(ShiftHandler const&) = delete;
  void operator=(ShiftHandler const&) = delete;

 private:
  bool _overrideActive = false;

  /*Private constructor to prevent creating multiple instances*/
  ShiftHandler() {}

  /*Delete assignment and copy privately to prevent any double ups inside*/
  ShiftHandler(ShiftHandler const&) = delete;
  void operator=(ShiftHandler const&) = delete;
  

};