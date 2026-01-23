#pragma once
#include <frc/geometry/Pose3d.h>

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

    constexpr int HOOD_MOTOR = 14; 
    constexpr int SHOOTER_MOTOR_1 = 15; 
    constexpr int SHOOTER_MOTOR_2 = 16;  
    constexpr int TURRET_MOTOR = 17; 

    constexpr int PIGEON_2 = 13; 
    constexpr int INTAKE = 14;
    constexpr int Indexer = 16;

}

namespace dio {
    constexpr int BRAKE_COAST_BUTTON = 0;
    constexpr int TURRET_ENCODER_1 = 2;
    constexpr int TURRET_ENCODER_2 = 1;
}

namespace pwm {
    constexpr int LED = 0;
}

namespace pose {
    constexpr frc::Translation3d HUB_POSE {4.625_m, 4.025_m, 1.823_m};
}