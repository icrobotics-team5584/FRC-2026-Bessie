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

    constexpr int HOOD_MOTOR = 96; 
    constexpr int SHOOTER_MOTOR_1 = 97; 
    constexpr int SHOOTER_MOTOR_2 = 98;  
    constexpr int TURRET_MOTOR = 17; 

    constexpr int PIGEON_2 = 91; 
    constexpr int INTAKE = 92;
    constexpr int Indexer = 93;
    constexpr int FEEDER = 94;
    constexpr int INTAKE_DEPLOY = 95;
}

namespace dio {
    constexpr int BRAKE_COAST_BUTTON = 0;
    constexpr int TURRET_ENCODER_1 = 2;
    constexpr int TURRET_ENCODER_2 = 1;
    constexpr int FEEDER_FULL_SENSOR = 4;
    constexpr int FEEDER_EMPTY_SENSOR = 5;
}

namespace pwm {
    constexpr int LED = 0;
}