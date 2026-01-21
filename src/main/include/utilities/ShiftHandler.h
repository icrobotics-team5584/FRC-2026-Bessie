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
        static RebuiltShift GetCurrentShift();
        static RebuiltShift GetWinningShift();
        static units::second_t GetTimeLeft();
        static std::string GetShiftName(RebuiltShift shift);
        static bool IsShift(RebuiltShift shift);
        static bool IsActiveShift();
};