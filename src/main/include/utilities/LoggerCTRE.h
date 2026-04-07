#pragma once

#include "utilities/Logger.h"
#include <ctre/phoenix6/TalonFX.hpp>

namespace Logger {
void LogFalcon(std::string name, ctre::phoenix6::hardware::TalonFX& talonFX);

void Log(std::string_view keyName, ctre::phoenix6::StatusSignal<double>& signal);
void Log(std::string keyName, ctre::phoenix6::StatusSignal<units::turn_t>& signal);
void Log(std::string keyName, ctre::phoenix6::StatusSignal<units::turns_per_second_t>& signal);
void Log(std::string keyName, ctre::phoenix6::StatusSignal<units::turns_per_second_squared_t>& signal);
void Log(std::string keyName, ctre::phoenix6::StatusSignal<units::volt_t>& signal);
void Log(std::string keyName, ctre::phoenix6::StatusSignal<units::ampere_t>& signal);
void Log(std::string keyName, ctre::phoenix6::StatusSignal<units::second_t>& signal);
void Log(std::string keyName, ctre::phoenix6::StatusSignal<units::celsius_t>& signal);
};  // namespace Logger
