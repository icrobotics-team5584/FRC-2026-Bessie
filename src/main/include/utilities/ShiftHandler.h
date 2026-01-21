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
        RebuiltShift GetShift();
        RebuiltShift GetWinningShift();
        units::second_t GetTimeLeft();
        bool IsShift(RebuiltShift shift);
};