#pragma once

namespace canid {
    constexpr int DRIVEBASE_FRONT_RIGHT_DRIVE = 1; 
    constexpr int DRIVEBASE_FRONT_RIGHT_TURN = 2;
    constexpr int DRIVEBASE_FRONT_RIGHT_ENCODER = 3; 

    constexpr int DRIVEBASE_FRONT_LEFT_DRIVE = 4;
    constexpr int DRIVEBASE_FRONT_LEFT_TURN = 5;
    constexpr int DRIVEBASE_FRONT_LEFT_ENCODER = 6;
        
    constexpr int DRIVEBASE_BACK_RIGHT_DRIVE = 7; 
    constexpr int DRIVEBASE_BACK_RIGHT_TURN = 8;
    constexpr int DRIVEBASE_BACK_RIGHT_ENCODER = 9; 

    constexpr int DRIVEBASE_BACK_LEFT_DRIVE = 10; 
    constexpr int DRIVEBASE_BACK_LEFT_TURN = 11;
    constexpr int DRIVEBASE_BACK_LEFT_ENCODER = 12; 

    constexpr int HOOD_MOTOR = 13; 
    constexpr int SHOOTER_MOTOR_1 = 14; 
    constexpr int SHOOTER_MOTOR_2 = 15; 
    constexpr int HOOD_MOTOR = 16; 
    constexpr int TURRET_MOTOR = 17; 

    constexpr int PIGEON_2 = 19; 

}

namespace dio {
    constexpr int BRAKE_COAST_BUTTON = 0;
}

namespace pwm {
    constexpr int LED = 0;
}