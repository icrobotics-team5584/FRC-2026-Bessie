#include <units/time.h>
#include <frc/DriverStation.h>

enum RebuiltShift {
    AUTON,
    TRANS,
    RED,
    BLUE,
    ENDGAME
};

class ShiftHandler {
    public:
        RebuiltShift GetCurrentShift();
        RebuiltShift GetWinningShift();
        units::second_t GetTimeLeft();
        bool IsShift(RebuiltShift shift);
};