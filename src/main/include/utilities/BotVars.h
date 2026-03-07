#pragma once

#include <frc/smartdashboard/SmartDashboard.h>

#include <filesystem>
#include <fstream>
#include <string>

namespace BotVars {

enum Robot { COMP, PRACTICE };
const inline std::string COMP_BOT_SERIAL = "00:80:2f:34:07:fe";
const inline std::string PRACTICE_BOT_SERIAL = "03239904";

Robot DetermineRobot();
Robot GetRobot();

template <typename T>
T Choose(T compBotValue, T practiceBotValue) {
  return GetRobot() == COMP ? compBotValue : practiceBotValue;
}

}  // namespace BotVars