#include "Constants.h"
#include <rev/config/SparkBaseConfig.h>
#include <units/time.h>
#include <ctre/phoenix6/signals/SpnEnums.hpp>

namespace NeoMotorConfig{
    constexpr double P = 16;
    constexpr double I = 0;
    constexpr double D = 8;
    constexpr double S = 0.6;
    constexpr double GEAR_RATIO = (56.0/8.0) * (370.0/34.0);

    constexpr int CURRENT_LIMT = 30;
    
    constexpr bool INVERTED = true;

    constexpr rev::spark::SparkBaseConfig::IdleMode IDLE_MODE = rev::spark::SparkBaseConfig::IdleMode::kBrake;
};

namespace KrakenMotorConfig{
    constexpr double P = 16;
    constexpr double I = 0;
    constexpr double D = 8;
    constexpr double S = 0.6;

    constexpr double GEAR_RATIO = (56.0/8.0) * (370.0/34.0);

    /* STATOR_CURRENT_LIMIT is the amount of current allowed inside the motor at
     * once. SUPPLY_CURRENT_LIMIT is true maximum current supplied to the motor
     * at once. SUPPLY_CURRENT_LOWER_LIMT is the typical supply current limit. 
     * However, it can exceed it, up to the true maximum for 
     * SUPPLY_CURRENT_LOWER_TIME seconds. */
    constexpr units::ampere_t SUPPLY_CURRENT_LIMT = 30_A;
    constexpr units::ampere_t SUPPLY_CURRENT_LOWER_LIMT = 20_A;
    constexpr units::ampere_t STATOR_CURRENT_LIMIT = 120_A;
    
    constexpr units::second_t SUPPLY_CURRENT_LOWER_TIME = 0.1_s;

    constexpr bool ENABLE_SUPPLY_CURRENT_LIMIT = true;
    constexpr bool ENABLE_STATOR_CURRENT_LIMIT = true;
    constexpr bool ENABLE_FOC = false;
    
    constexpr ctre::phoenix6::signals::InvertedValue INVERTED = ctre::phoenix6::signals::InvertedValue::Clockwise_Positive;
    constexpr ctre::phoenix6::signals::NeutralModeValue IDLE_MODE = ctre::phoenix6::signals::NeutralModeValue::Brake;
};